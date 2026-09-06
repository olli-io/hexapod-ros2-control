"""What the teleop server does with a request that is not the webapp's.

Pure string work — no aiohttp, no sockets. These pin the rule that makes a
phone joining the robot's hotspot end up holding the controller without anyone
telling it an address: every path that is not a webapp file is somebody who
wants the app, so it goes to the app rather than to a 404.
"""
import json
import re
from pathlib import Path
from urllib.parse import urljoin, urlparse

import pytest

from hexa_webteleop.captive_portal import request_path, static_filename


# --- request paths --------------------------------------------------------


def test_the_decoration_a_client_adds_does_not_change_a_request():
    """Connectivity probes append cache-busting queries, and clients add
    trailing slashes; neither may make a known path read as an unknown one."""
    assert request_path("/generate_204?ts=1712345") == "/generate_204"
    assert request_path("/main.js#frag") == "/main.js"
    assert request_path("/ncsi.txt/") == "/ncsi.txt"


def test_the_root_survives_normalisation():
    """Stripping trailing slashes must not turn "/" into an empty path."""
    assert request_path("/") == "/"
    assert request_path("/?x=1") == "/"


# --- static files ---------------------------------------------------------


def test_a_flat_asset_name_is_served_from_the_web_directory():
    """The rule, not the manifest: the single-file build ships only index.html,
    but the server still has to accept a flat name for anything too large for
    the bundler to inline.
    """
    assert static_filename("/main.js") == "main.js"
    assert static_filename("/styles.css") == "styles.css"


@pytest.mark.parametrize(
    "target",
    [
        "/",  # the app itself, served by its own route
        "/some/other/site",  # a stale tab, via the hotspot's wildcard DNS
        "/library/test/success.html",  # macOS' probe, nested besides
    ],
)
def test_nothing_nested_is_an_asset(target):
    """None means "redirect to the controller" — the webapp is flat, so a path
    with a directory in it belongs to somebody else entirely."""
    assert static_filename(request_path(target)) is None


#: The flat paths an OS asks for to decide whether a network has internet.
#: Nothing here decides behaviour — a probe is served like any other name, which
#: is to say it is redirected, because no such file exists. That is the point of
#: the test below.
_PROBE_NAMES = (
    "generate_204",  # Android, ChromeOS
    "gen_204",  # Android (older)
    "hotspot-detect.html",  # iOS, macOS — captive.apple.com
    "success.html",  # macOS
    "connecttest.txt",  # Windows 10/11
    "ncsi.txt",  # Windows (legacy NCSI)
    "redirect",  # Windows — msftconnecttest.com
    "canonical.html",  # Firefox
    "success.txt",  # Firefox — detectportal
    "check_network_status.txt",  # GNOME
    "nm-check.txt",  # NetworkManager
)


# What setup.py installs into share/hexa_webteleop/web, and therefore the only
# directory the static server can serve out of. The bundle is committed and is a
# single inlined index.html; a build is `npm run build` in web/.
_DIST = Path(__file__).resolve().parents[1] / "web" / "dist"


@pytest.mark.parametrize("name", _PROBE_NAMES)
def test_the_webapp_ships_no_file_an_os_probes_for(name):
    """A phone opens its sign-in browser on the controller *because* its probe
    goes unanswered. An asset added under one of these names would answer it,
    the phone would call the hotspot an ordinary network, and the popup that
    saves anyone having to be told an address would quietly stop happening.
    """
    assert not (_DIST / name).exists()


def test_the_committed_bundle_is_a_built_one():
    """The bundle is a build artefact that lives in git, because the ARM64 robot
    image builds from a bare checkout with no node in it. A rebuild that was
    never committed ships a robot with no UI, so the suite is where that is
    caught.

    The build is a single file — `vite-plugin-singlefile` inlines the script and
    the stylesheet — so a built page carries the whole app. The unbuilt source
    `web/index.html`, which is a stub pointing at `/src/main.tsx`, is what the
    two assertions tell apart.
    """
    index = _DIST / "index.html"
    assert index.is_file()
    html = index.read_text()
    assert "/src/main.tsx" not in html, "web/index.html was copied in unbuilt"
    assert len(html) > 50_000, "the app is inlined; a small page is not a build"


def test_every_shipped_asset_is_reachable():
    """The bundle must stay flat. ``static_filename`` refuses any nested path and
    ``_handle_get`` answers that with a 302 to "/" rather than a 404, so a file
    under an assets/ subdirectory would fail *silently* — the browser would be
    handed the HTML page in place of the script it asked for.
    """
    for entry in _DIST.iterdir():
        assert entry.is_file(), f"{entry.name} is a directory; the bundle must be flat"
        assert static_filename(request_path("/" + entry.name)) == entry.name


def test_the_index_references_only_flat_assets():
    """Same rule, from the other end: whatever the built index.html points at has
    to be a name the server will serve, and has to be there.

    Existence is half the test because of how a miss fails. A manifest or an
    icon the build never emitted is still a flat, acceptable *name*, so the
    server takes the request, finds no file, and 302s it to "/" — handing the
    browser this very page in place of the PNG it asked for. Nothing errors and
    nothing logs; the icon is just silently wrong. Asserting the file exists is
    what turns that into a red test.
    """
    # Only the markup, which here means everything before the inlined bundle:
    # the whole app is a <script> in this same file, and minified JS is full of
    # fragments like href="'+St(e)+'" that are not references to anything. The
    # page's own references — the manifest, the icons — are all head tags, and
    # the head ends before the first script.
    markup = (_DIST / "index.html").read_text().split("<script", 1)[0]
    for ref in re.findall(r'(?:src|href)="([^"]+)"', markup):
        # The page is served from "/", so that is what a relative href resolves
        # against. An off-site URL is somebody else's problem.
        if urlparse(ref).scheme or ref.startswith("//"):
            continue
        name = static_filename(request_path(urljoin("/", ref)))
        assert name is not None, ref
        assert (_DIST / name).is_file(), f"{ref} is referenced but not shipped"


# --- the installable app shell --------------------------------------------


def _manifest() -> dict:
    return json.loads((_DIST / "manifest.webmanifest").read_text())


def test_the_manifest_ships_and_parses():
    """Without it the page is a page; with it the same page is something iOS
    will put on a home screen and launch without browser chrome. It comes from
    web/public/, which Vite copies into dist/ verbatim — it cannot be dropped
    into dist/ by hand, because the build empties that directory first.
    """
    assert (_DIST / "manifest.webmanifest").is_file()
    assert _manifest()["name"]


def test_the_manifest_is_scoped_to_the_root():
    """The server serves exactly one document, at "/", and 302s every other path
    to it — which is what makes a joining phone declare a captive portal. A
    start_url or scope below the root would name a path that redirects, so the
    installed app would launch on a redirect back to where it started, and any
    browser checking scope would put its own chrome back on. The router runs on
    a hash history for the same reason; there is no second document to point at.
    """
    manifest = _manifest()
    assert manifest["start_url"] == "/"
    assert manifest["scope"] == "/"
    assert manifest["display"] == "standalone"


def test_every_manifest_icon_is_flat_and_shipped():
    """Same silent-302 hazard as the index references above, one indirection
    further out: nothing in the HTML mentions these, so only reading the
    manifest catches an icon that is nested or missing.
    """
    icons = _manifest()["icons"]
    assert icons, "a manifest with no icons is not installable"
    for icon in icons:
        name = static_filename(request_path(icon["src"]))
        assert name is not None, icon["src"]
        assert (_DIST / name).is_file(), f"{icon['src']} is in the manifest but not shipped"


def test_the_bundle_ships_no_service_worker():
    """There is deliberately no service worker, and this is where that decision
    is written down rather than only argued for in the README.

    It could not work: service workers register only in a secure context, and
    every address the robot answers on is plain HTTP. It should not work
    either — the server sets `Cache-Control: no-store` on everything so a phone
    can never run last week's UI against today's socket protocol, and a
    fetch-handling worker would both undo that and start answering the
    connectivity probes whose going unanswered is what opens the captive portal.
    """
    for entry in _DIST.iterdir():
        assert "serviceworker" not in entry.name.lower().replace("-", "").replace(
            "_", ""
        ), entry.name
        assert entry.name not in ("sw.js", "service-worker.js"), entry.name


@pytest.mark.parametrize(
    "target", ["/../config/webteleop.yaml", "/..", "/.env", "/foo/../../etc/passwd"]
)
def test_no_path_can_escape_the_web_directory(target):
    assert static_filename(request_path(target)) is None
