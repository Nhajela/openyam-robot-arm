"""Browser smoke test of the phone page's camera modes (Robot Eyes, Claw Cam, Grab's feed) against a
MOCK open play (never the real arm). Usage: python cammodes_smoke.py https://127.0.0.1:8460/?k=<key> <out dir>
Needs Playwright (python) and a Chromium (CHROME_PATH)."""
import os, sys, time, json
from playwright.sync_api import sync_playwright

url, out = sys.argv[1], sys.argv[2]
q = lambda page: page.evaluate("() => window.openplay.S.arm && window.openplay.S.arm.q")
errors, res = [], {}
with sync_playwright() as p:
    b = p.chromium.launch(executable_path=os.environ.get("CHROME_PATH") or None, args=["--ignore-certificate-errors"])
    ctx = b.new_context(viewport={"width": 390, "height": 844}, device_scale_factor=2, has_touch=True, is_mobile=True, ignore_https_errors=True)
    page = ctx.new_page()
    page.on("pageerror", lambda e: errors.append(str(e)))
    page.on("console", lambda m: m.type == "error" and errors.append(m.text))
    page.goto(url)
    page.wait_for_function("() => window.openplay && window.openplay.S.welcome")
    page.fill("#name", "Asha"); page.click("#joinBtn")
    page.wait_for_selector("#view-modes:not([hidden])", timeout=15000)
    res["cards"] = page.eval_on_selector_all(".mcard", "els => els.map(e => e.dataset.mode)")
    page.screenshot(path=f"{out}/1-modes.png", full_page=True)

    # Robot Eyes
    page.click('.mcard[data-mode="eyes"]'); page.click("#gsGo")
    page.wait_for_function("() => window.openplay.S.arm && !window.openplay.S.arm.busy && Math.abs(window.openplay.S.arm.q[1]-0.5)<0.05", timeout=20000)
    time.sleep(1.5)
    q0 = q(page)
    bb = page.locator("#panel-eyes canvas").bounding_box()
    cx, cy = bb["x"] + bb["width"] / 2, bb["y"] + bb["height"] / 2
    page.mouse.move(cx, cy); page.mouse.down()
    for i in range(1, 11): page.mouse.move(cx + 6 * i, cy - 4 * i); time.sleep(0.02)
    time.sleep(1.2); page.screenshot(path=f"{out}/2-eyes-drag.png"); page.mouse.up(); time.sleep(0.6)
    q1 = q(page)
    res["eyes_dq"] = [round(b_ - a, 3) for a, b_ in zip(q0, q1)]
    page.click('#panel-eyes .chip:has-text("Thermal")')
    page.click("#panel-eyes button:has-text('SNAP')")
    page.wait_for_selector(".cam-photo:not([hidden])", timeout=6000)
    page.screenshot(path=f"{out}/3-eyes-photo.png")
    page.click(".cam-photo button:has-text('Close')")

    # Claw Cam
    page.click("#modesBtn"); page.click('.mcard[data-mode="clawcam"]'); page.click("#gsGo")
    page.wait_for_function("() => window.openplay.S.view === 'play' && window.openplay.S.arm && !window.openplay.S.arm.busy", timeout=20000)
    time.sleep(1.0)
    q0 = q(page)
    bb = page.locator("#panel-clawcam canvas").bounding_box()
    cx, cy = bb["x"] + bb["width"] / 2, bb["y"] + bb["height"] / 2
    page.mouse.move(cx, cy); page.mouse.down()
    for i in range(1, 11): page.mouse.move(cx, cy - 6 * i); time.sleep(0.02)
    time.sleep(1.2); page.screenshot(path=f"{out}/4-clawcam-steer.png"); page.mouse.up(); time.sleep(0.6)
    q1 = q(page)
    res["clawcam_steer_dq"] = [round(b_ - a, 3) for a, b_ in zip(q0, q1)]
    d = page.locator(".cam-drop").bounding_box()
    page.mouse.move(d["x"] + d["width"] / 2, d["y"] + d["height"] / 2); page.mouse.down(); time.sleep(1.2)
    page.screenshot(path=f"{out}/5-clawcam-drop.png"); page.mouse.up()
    page.wait_for_function("() => window.openplay.S.arm.busy === 'ready'", timeout=5000)
    res["after_drop"] = page.evaluate("() => ({busy: window.openplay.S.arm.busy, grip_target: window.openplay.S.arm.grip_target ?? window.openplay.S.arm.grip})")

    # Grab: the live feed above the spots (if Grab is offered)
    page.wait_for_function("() => !window.openplay.S.arm.busy", timeout=20000)
    page.click("#modesBtn")
    if page.locator('.mcard[data-mode="grab"]').count():
        page.click('.mcard[data-mode="grab"]'); page.click("#gsGo"); time.sleep(2)
        res["grab_has_cam"] = page.locator("#grabCam .cam-box").count() == 1
        page.screenshot(path=f"{out}/6-grab.png", full_page=True)
    else:
        res["grab_has_cam"] = "grab not offered"
    page.click("#modesBtn")
    res["cam_box_detached_on_modes"] = page.locator(".cam-box").count() == 0
    b.close()
res["errors"] = errors
print(json.dumps(res, indent=1))
