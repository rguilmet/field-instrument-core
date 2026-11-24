#include "fic_web_portal_html.h"

#include <algorithm>
#include <ctype.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <time.h>
#include <vector>

namespace {

struct WebFileEntry {
    String filename;
    String item_rel;
    size_t size;
    time_t mtime;
    String dt_str;
    bool   is_bmp;
};

bool parse_filename_datetime(const String &name, time_t &out_time, String &out_str)
{
    const char *str = name.c_str();
    const int len = name.length();
    for (int i = 0; i <= len - 19; i++) {
        if (isdigit((unsigned char)str[i]) && isdigit((unsigned char)str[i + 1]) &&
            isdigit((unsigned char)str[i + 2]) && isdigit((unsigned char)str[i + 3]) &&
            str[i + 4] == '-' &&
            isdigit((unsigned char)str[i + 5]) && isdigit((unsigned char)str[i + 6]) &&
            str[i + 7] == '-' &&
            isdigit((unsigned char)str[i + 8]) && isdigit((unsigned char)str[i + 9]) &&
            str[i + 10] == '_' &&
            isdigit((unsigned char)str[i + 11]) && isdigit((unsigned char)str[i + 12]) &&
            str[i + 13] == '-' &&
            isdigit((unsigned char)str[i + 14]) && isdigit((unsigned char)str[i + 15]) &&
            str[i + 16] == '-' &&
            isdigit((unsigned char)str[i + 17]) && isdigit((unsigned char)str[i + 18])) {

            int y   = atoi(name.substring(i, i + 4).c_str());
            int m   = atoi(name.substring(i + 5, i + 7).c_str());
            int d   = atoi(name.substring(i + 8, i + 10).c_str());
            int h   = atoi(name.substring(i + 11, i + 13).c_str());
            int min = atoi(name.substring(i + 14, i + 16).c_str());
            int s   = atoi(name.substring(i + 17, i + 19).c_str());

            if (y >= 2020 && m >= 1 && m <= 12 && d >= 1 && d <= 31 &&
                h >= 0 && h <= 23 && min >= 0 && min <= 59 && s >= 0 && s <= 59) {
                struct tm tm_buf;
                memset(&tm_buf, 0, sizeof(tm_buf));
                tm_buf.tm_year = y - 1900;
                tm_buf.tm_mon  = m - 1;
                tm_buf.tm_mday = d;
                tm_buf.tm_hour = h;
                tm_buf.tm_min  = min;
                tm_buf.tm_sec  = s;
                out_time = mktime(&tm_buf);

                char buf[32];
                snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", y, m, d, h, min, s);
                out_str = String(buf);
                return true;
            }
        }
    }
    return false;
}

String format_file_size(size_t bytes)
{
    if (bytes < 1024U) {
        return String(bytes) + " B";
    }
    char buf[32];
    if (bytes < (1024U * 1024U)) {
        snprintf(buf, sizeof(buf), "%.1f KB", (float)bytes / 1024.0f);
    } else {
        snprintf(buf, sizeof(buf), "%.2f MB", (float)bytes / (1024.0f * 1024.0f));
    }
    return String(buf);
}

void collect_files_recursive(std::vector<WebFileEntry> &files, const char *vfs_base, const char *rel_sub)
{
    if (!vfs_base || vfs_base[0] == '\0') {
        return;
    }
    String full_dir = String(vfs_base) + String(rel_sub);
    DIR *dir = opendir(full_dir.c_str());
    if (!dir) {
        return;
    }

    struct dirent *ent;
    while ((ent = readdir(dir)) != NULL) {
        String filename = String(ent->d_name);
        if (filename == "." || filename == "..") {
            continue;
        }

        String item_vfs = full_dir + "/" + filename;
        String item_rel = String(rel_sub) + "/" + filename;
        if (!item_rel.startsWith("/")) {
            item_rel = "/" + item_rel;
        }

        struct stat st;
        if (stat(item_vfs.c_str(), &st) == 0) {
            if (S_ISDIR(st.st_mode)) {
                collect_files_recursive(files, vfs_base, item_rel.c_str());
            } else if (S_ISREG(st.st_mode)) {
                WebFileEntry entry;
                entry.filename = filename;
                entry.item_rel = item_rel;
                entry.size     = st.st_size;
                entry.is_bmp   = filename.endsWith(".bmp");

                time_t parsed_time = 0;
                String parsed_dt = "";
                if (parse_filename_datetime(filename, parsed_time, parsed_dt)) {
                    entry.mtime  = parsed_time;
                    entry.dt_str = parsed_dt;
                } else if (st.st_mtime > 946684800) {
                    entry.mtime = st.st_mtime;
                    struct tm *tm_info = localtime(&st.st_mtime);
                    char buf[32];
                    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", tm_info);
                    entry.dt_str = String(buf);
                } else {
                    entry.mtime  = 0;
                    entry.dt_str = "-";
                }
                files.push_back(entry);
            }
        }
    }
    closedir(dir);
}

void sort_web_files(std::vector<WebFileEntry> &files)
{
    std::sort(files.begin(), files.end(), [](const WebFileEntry &a, const WebFileEntry &b) {
        if (a.mtime != b.mtime) {
            return a.mtime > b.mtime;
        }
        return a.item_rel > b.item_rel;
    });
}

String generate_dir_table_html(const char *title, const char *fs_param, const char *vfs_path, const char *table_id)
{
    std::vector<WebFileEntry> files;
    collect_files_recursive(files, vfs_path, "");
    sort_web_files(files);

    String out;
    out.reserve(files.size() * 260U + 512U);
    out += "<h2>" + String(title) + " <span class='badge'>" + String(files.size()) + " files</span></h2>";

    if (files.empty()) {
        out += "<p style='color:#94a3b8;font-style:italic;margin:10px 0;'>No files found in storage.</p>";
        return out;
    }

    out += "<div class='sort-bar'>";
    out += "<span style='color:#94a3b8;font-size:12px;margin-right:4px;'>Quick Sort:</span>";
    out += "<button type='button' class='btn-sort' onclick=\"sortTable('" + String(table_id) + "', 1, true, 'desc')\">Newest First</button>";
    out += "<button type='button' class='btn-sort' onclick=\"sortTable('" + String(table_id) + "', 1, true, 'asc')\">Oldest First</button>";
    out += "<button type='button' class='btn-sort' onclick=\"sortTable('" + String(table_id) + "', 0, false, 'asc')\">Name (A-Z)</button>";
    out += "<button type='button' class='btn-sort' onclick=\"sortTable('" + String(table_id) + "', 0, false, 'desc')\">Name (Z-A)</button>";
    out += "<button type='button' class='btn-sort' onclick=\"sortTable('" + String(table_id) + "', 2, true, 'desc')\">Largest First</button>";
    out += "</div>";

    out += "<div class='tbl-wrap'><table id='" + String(table_id) + "'>";
    out += "<thead><tr>";
    out += "<th onclick=\"toggleSort('" + String(table_id) + "', 0, false)\" title=\"Click to sort by Filename\">File Name / Path <span class='sort-icon' id='" + String(table_id) + "_i0'></span></th>";
    out += "<th onclick=\"toggleSort('" + String(table_id) + "', 1, true)\" title=\"Click to sort by Date & Time\">Date &amp; Time <span class='sort-icon' id='" + String(table_id) + "_i1'> [v]</span></th>";
    out += "<th onclick=\"toggleSort('" + String(table_id) + "', 2, true)\" title=\"Click to sort by Size\">Size <span class='sort-icon' id='" + String(table_id) + "_i2'></span></th>";
    out += "<th>Actions</th>";
    out += "</tr></thead><tbody>";

    for (const auto &f : files) {
        out += "<tr>";
        out += "<td data-val=\"" + f.item_rel + "\">";
        if (f.is_bmp) {
            out += "<a href=\"/view?fs=" + String(fs_param) + "&file=" + f.item_rel + "\" target=\"_blank\">" + f.item_rel + "</a>";
        } else {
            out += "<a href=\"#\" onclick=\"dlFile('/download?fs=" + String(fs_param) + "&file=" + f.item_rel + "', '" + f.filename + "'); return false;\" title=\"Download securely via in-memory Blob\">" + f.item_rel + "</a>";
        }
        out += "</td>";
        out += "<td data-val=\"" + String((uint32_t)f.mtime) + "\">" + f.dt_str + "</td>";
        out += "<td data-val=\"" + String(f.size) + "\" title=\"" + String(f.size) + " bytes\">" + format_file_size(f.size) + "</td>";
        out += "<td class='actions'>";
        if (f.is_bmp) {
            out += "<a href=\"/view?fs=" + String(fs_param) + "&file=" + f.item_rel + "\" target=\"_blank\">[View]</a>";
        } else {
            out += "<a href=\"/download?fs=" + String(fs_param) + "&file=" + f.item_rel + "&raw=1\" target=\"_blank\" title=\"View raw text in browser\">[View]</a>";
            out += "<a href=\"#\" onclick=\"dlFile('/download?fs=" + String(fs_param) + "&file=" + f.item_rel + "', '" + f.filename + "'); return false;\" title=\"Download file\">[Download]</a>";
        }
        out += "<a href=\"/delete?fs=" + String(fs_param) + "&file=" + f.item_rel + "\" class=\"del-btn\" onclick=\"return confirm('Delete " + f.filename + "?');\">[Delete]</a>";
        out += "</td></tr>";
    }

    out += "</tbody></table></div>";
    return out;
}

} // namespace

String fic_web_render_portal_page(const FicWebPortalCfg_t *cfg)
{
    String title_str = (cfg && cfg->portal_title) ? String(cfg->portal_title) : String("Field Instrument Portal");
    if (cfg && cfg->version_str && cfg->version_str[0] != '\0') {
        title_str += " " + String(cfg->version_str);
    }

    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>" + title_str + "</title><style>"
        "body{font-family:-apple-system,BlinkMacSystemFont,Segoe UI,Roboto,sans-serif;margin:20px;background:#12181b;color:#e0e6ed;}"
        "h1{color:#00ffaa;margin-bottom:6px;}"
        "h2{color:#00d8ff;border-bottom:1px solid #334;padding-bottom:6px;margin-top:28px;}"
        "a{color:#38bdf8;text-decoration:none;}"
        "a:hover{text-decoration:underline;}"
        ".badge{background:#0284c7;color:#fff;padding:2px 8px;border-radius:4px;font-size:12px;font-weight:normal;margin-left:8px;}"
        ".up{margin-bottom:20px;padding:15px;background:#1e293b;border-radius:8px;}"
        "input[type=submit],button{background:#00ffaa;color:#000;font-weight:bold;border:none;padding:8px 16px;border-radius:4px;cursor:pointer;}"
        ".sort-bar{margin:10px 0;display:flex;flex-wrap:wrap;gap:6px;align-items:center;}"
        ".btn-sort{background:#243447;color:#38bdf8;font-size:12px;font-weight:600;padding:5px 10px;border-radius:4px;border:1px solid #3b4f68;}"
        ".btn-sort:hover{background:#334e68;color:#00ffaa;}"
        ".tbl-wrap{overflow-x:auto;margin-top:8px;border-radius:8px;border:1px solid #233142;}"
        "table{width:100%;border-collapse:collapse;font-family:monospace;font-size:13px;background:#18222d;}"
        "th,td{padding:9px 12px;text-align:left;border-bottom:1px solid #233142;white-space:nowrap;}"
        "th{background:#1e293b;color:#00d8ff;cursor:pointer;user-select:none;}"
        "th:hover{color:#00ffaa;background:#26354a;}"
        "tr:hover{background:#1f2c3a;}"
        ".sort-icon{font-size:11px;margin-left:4px;color:#94a3b8;}"
        ".actions a{margin-right:8px;}"
        ".del-btn{color:#f87171 !important;}"
        ".del-btn:hover{color:#ef4444 !important;}"
        "</style><script>"
        "function dlFile(url, name){"
        "  fetch(url).then(function(r){if(!r.ok) throw new Error('Download failed: ' + r.status); return r.blob();})"
        "  .then(function(blob){"
        "    var mime = name.endsWith('.csv') ? 'text/csv;charset=utf-8' : (name.endsWith('.json') ? 'application/json;charset=utf-8' : 'text/plain;charset=utf-8');"
        "    var safeBlob = new Blob([blob], {type: mime});"
        "    var a = document.createElement('a'); a.href = URL.createObjectURL(safeBlob); a.download = name;"
        "    document.body.appendChild(a); a.click();"
        "    setTimeout(function(){URL.revokeObjectURL(a.href); a.remove();}, 2000);"
        "  }).catch(function(err){window.location.href = url;});"
        "}"
        "function toggleSort(tableId, colIdx, isNum) {"
        "  var tbl = document.getElementById(tableId);"
        "  if (!tbl) return;"
        "  var curDir = tbl.getAttribute('data-dir-' + colIdx);"
        "  var nextDir = (curDir === 'asc') ? 'desc' : 'asc';"
        "  sortTable(tableId, colIdx, isNum, nextDir);"
        "}"
        "function sortTable(tableId, colIdx, isNum, forceDir) {"
        "  var tbl = document.getElementById(tableId);"
        "  if (!tbl) return;"
        "  var tbody = tbl.querySelector('tbody');"
        "  if (!tbody) return;"
        "  var rows = Array.from(tbody.querySelectorAll('tr'));"
        "  var dir = forceDir || ((tbl.getAttribute('data-dir-' + colIdx) === 'asc') ? 'desc' : 'asc');"
        "  tbl.setAttribute('data-dir-' + colIdx, dir);"
        "  for (var c = 0; c < 3; c++) {"
        "    var icon = document.getElementById(tableId + '_i' + c);"
        "    if (icon) icon.innerText = (c === colIdx) ? (dir === 'asc' ? ' [^]' : ' [v]') : '';"
        "  }"
        "  rows.sort(function(a, b) {"
        "    var ca = a.cells[colIdx], cb = b.cells[colIdx];"
        "    var va = ca ? (ca.getAttribute('data-val') || ca.innerText.trim()) : '';"
        "    var vb = cb ? (cb.getAttribute('data-val') || cb.innerText.trim()) : '';"
        "    if (isNum) {"
        "      var na = parseFloat(va) || 0, nb = parseFloat(vb) || 0;"
        "      return (dir === 'asc') ? (na - nb) : (nb - na);"
        "    }"
        "    return (dir === 'asc') ? va.localeCompare(vb) : vb.localeCompare(va);"
        "  });"
        "  rows.forEach(function(r) { tbody.appendChild(r); });"
        "}"
        "</script></head><body><h1>" + title_str + "</h1>";

    auto upload_html = [](const char *fs_param, const char *title) -> String {
        return "<div class='up'><h3>Upload to " + String(title) + "</h3><form method='POST' action='/upload?fs=" + String(fs_param) + "' enctype='multipart/form-data'><input type='file' name='f' style='color:#fff'> <input type='submit' value='Upload'></form></div>";
    };

    if (cfg && cfg->primary_fs != nullptr) {
        const char *lbl  = cfg->primary_label    ? cfg->primary_label    : "Primary Storage";
        const char *vfsp = cfg->primary_vfs_path ? cfg->primary_vfs_path : "/sdcard";
        html += upload_html("sd", lbl);
        html += generate_dir_table_html((String(lbl) + " (" + String(vfsp) + ")").c_str(), "sd", vfsp, "tbl_sd");
    }

    if (cfg && cfg->fallback_fs != nullptr) {
        const char *lbl  = cfg->fallback_label    ? cfg->fallback_label    : "Internal Flash (FFat)";
        const char *vfsp = cfg->fallback_vfs_path ? cfg->fallback_vfs_path : "/ffat";
        html += upload_html("ffat", lbl);
        html += generate_dir_table_html((String(lbl) + " (" + String(vfsp) + ")").c_str(), "ffat", vfsp, "tbl_ffat");
    }

    html += "</body></html>";
    return html;
}

String fic_web_render_view_image_page(const String &path, const String &filename, const String &fs_type)
{
    String stem = filename.endsWith(".bmp") ? filename.substring(0, filename.length() - 4) : filename;
    String html = "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'><title>View Image</title></head><body style='background:#12181b;color:white;text-align:center;font-family:sans-serif;padding:20px;'>";
    html += "<h2 style='color:#00ffaa;'>" + filename + "</h2>";
    html += "<button onclick='dl()' style='padding:12px 24px;font-size:16px;background:#00d8ff;color:#000;font-weight:bold;border:none;border-radius:6px;cursor:pointer;'>Save Securely as PNG</button>";
    html += "<p style='color:#94a3b8;font-size:13px;margin-top:10px;margin-bottom:20px;'>Converts internal BMP to PNG via client browser to bypass HTTP download warnings.</p>";
    html += "<img id='img' src='/download?fs=" + fs_type + "&file=" + path + "' style='max-width:100%; border:2px solid #334; border-radius:8px;'><br><br>";
    html += "<script>function dl(){";
    html += "var img = document.getElementById('img');";
    html += "var c = document.createElement('canvas'); c.width = img.naturalWidth; c.height = img.naturalHeight;";
    html += "c.getContext('2d').drawImage(img, 0, 0);";
    html += "var a = document.createElement('a'); a.download = '" + stem + ".png';";
    html += "a.href = c.toDataURL('image/png'); a.click();";
    html += "}</script></body></html>";
    return html;
}
