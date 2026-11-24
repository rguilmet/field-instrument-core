#ifndef FIC_WEB_PORTAL_HTML_H
#define FIC_WEB_PORTAL_HTML_H

#include <Arduino.h>
#include "fic_web_portal.h"

String fic_web_render_portal_page(const FicWebPortalCfg_t *cfg);
String fic_web_render_view_image_page(const String &path, const String &filename, const String &fs_type);

#endif // FIC_WEB_PORTAL_HTML_H
