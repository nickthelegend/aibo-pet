"""server.py — serves web/ for the deployed site.

Deliberately stdlib only: the whole project is pure Python with two deps
(numpy, shapely) that the SITE does not need, so the deployed image installs
nothing. Binds $PORT because that is how Railway hands you a port.

Not a general-purpose server. It serves one directory, refuses to walk out of
it, and sets the handful of headers that actually matter here: the correct
type for .glb so the browser does not download it as a file, and a short
cache on the model so a rebuild is picked up rather than pinned.
"""
from __future__ import annotations

import os
import sys
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")


class Handler(SimpleHTTPRequestHandler):
    extensions_map = {
        **SimpleHTTPRequestHandler.extensions_map,
        ".glb": "model/gltf-binary",
        ".svg": "image/svg+xml",
        ".js": "text/javascript",
        ".json": "application/json",
        ".css": "text/css",
    }

    def end_headers(self):
        p = self.path.split("?")[0]
        if p.endswith((".glb", ".json")):
            # short: a redeploy should actually reach people
            self.send_header("Cache-Control", "public, max-age=300")
        elif p.endswith((".css", ".js", ".svg")):
            self.send_header("Cache-Control", "public, max-age=3600")
        self.send_header("X-Content-Type-Options", "nosniff")
        super().end_headers()

    def log_message(self, fmt, *args):
        sys.stdout.write("%s %s\n" % (self.address_string(), fmt % args))
        sys.stdout.flush()


def main():
    port = int(os.environ.get("PORT", "8080"))
    httpd = ThreadingHTTPServer(("0.0.0.0", port),
                                partial(Handler, directory=ROOT))
    print(f"aibo site on :{port} from {ROOT}", flush=True)
    httpd.serve_forever()


if __name__ == "__main__":
    main()
