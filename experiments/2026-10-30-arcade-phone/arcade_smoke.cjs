// Smoke test + screenshots of the arcade phone page against a mock daemon (never the real arm).
//   node arcade_smoke.cjs https://127.0.0.1:<phone port>/arcade?k=<key> <out dir>
// CHROME_PATH=<chrome binary> if this Playwright has no browser of its own; NODE_PATH to find playwright.
// Joins, presses start, steers by dragging on the map, holds DROP, lets go, and checks that the
// mock arm's tip moved the right way. Screenshots: lobby, intro, play, drop, menu, game over.
const { chromium } = require('playwright');
const [url, out = '.'] = process.argv.slice(2);
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
(async () => {
  const browser = await chromium.launch({ args: ['--ignore-certificate-errors'], executablePath: process.env.CHROME_PATH || undefined });
  const ctx = await browser.newContext({ viewport: { width: 390, height: 844 }, deviceScaleFactor: 3, hasTouch: true, isMobile: true, ignoreHTTPSErrors: true });
  const page = await ctx.newPage();
  const errors = [];
  page.on('pageerror', (e) => errors.push(String(e)));
  page.on('console', (m) => { if (m.type() === 'error') errors.push(m.text()); });
  await page.goto(url);
  await page.waitForFunction(() => window.openplay && window.openplay.S.welcome);
  await page.fill('#name', 'Asha');
  await page.screenshot({ path: `${out}/1-lobby.png` });
  await page.click('#joinBtn');
  await page.waitForSelector('#intro:not([hidden])', { timeout: 15000 });
  await page.screenshot({ path: `${out}/2-intro.png` });
  await page.click('#startBtn');
  await page.waitForFunction(() => window.openplay.S.started, null, { timeout: 15000 });
  await page.waitForFunction(() => window.openplay.S.arm && !window.openplay.S.arm.busy, null, { timeout: 20000 });
  await sleep(600);
  const t0 = await page.evaluate(() => window.openplay.tip());
  // steer: press in the arena, drag up (away from the base) and hold
  const box = await page.locator('#arena').boundingBox();
  const cx = box.x + box.width / 2, cy = box.y + box.height * 0.6;
  await page.mouse.move(cx, cy); await page.mouse.down();
  for (let i = 1; i <= 10; i++) { await page.mouse.move(cx + 2 * i, cy - 6 * i); await sleep(20); }
  await sleep(1500);
  await page.screenshot({ path: `${out}/3-steering.png` });
  await page.mouse.up();
  await sleep(500);
  const t1 = await page.evaluate(() => window.openplay.tip());
  // drop: hold the DROP button
  const z0 = (await page.evaluate(() => window.openplay.tip()))[2];
  const d = await page.locator('#dropBtn').boundingBox();
  await page.mouse.move(d.x + d.width / 2, d.y + d.height / 2); await page.mouse.down();
  await sleep(1500);
  await page.screenshot({ path: `${out}/4-dropping.png` });
  const z1 = (await page.evaluate(() => window.openplay.tip()))[2];
  await page.mouse.up();
  await page.waitForFunction(() => window.openplay.S.arm && window.openplay.S.arm.busy === 'ready', null, { timeout: 5000 }).catch(() => {});
  const busyAfterDrop = await page.evaluate(() => window.openplay.S.arm && window.openplay.S.arm.busy);
  await page.screenshot({ path: `${out}/5-grab-return.png` });
  await page.waitForFunction(() => window.openplay.S.arm && !window.openplay.S.arm.busy, null, { timeout: 20000 });
  const grip = await page.evaluate(() => window.openplay.S.arm.grip);
  // fly to a star (place the drive by hand: steer toward the nearest star for a few seconds)
  const before = await page.evaluate(() => window.openplay.G.score);
  for (let k = 0; k < 6; k++) {
    const st = await page.evaluate(() => { const o = window.openplay; const t = o.tip(); const s = o.G.stars.filter((x) => !x.gold)[0]; return s && t ? { dx: s.x - t[0], dy: s.y - t[1] } : null; });
    if (!st || Math.hypot(st.dx, st.dy) < 0.03) break;
    const n = Math.hypot(st.dx, st.dy);
    // base frame -> screen (unflipped): +x up, +y left
    const sx = -st.dy / n * 60, sy = -st.dx / n * 60;
    await page.mouse.move(cx, cy); await page.mouse.down(); await page.mouse.move(cx + sx, cy + sy);
    await sleep(Math.min(2500, n / 0.08 * 1000)); await page.mouse.up(); await sleep(300);
  }
  const after = await page.evaluate(() => window.openplay.G.score);
  await page.screenshot({ path: `${out}/6-score.png` });
  await page.click('#pauseBtn'); await sleep(200);
  await page.screenshot({ path: `${out}/7-menu.png` });
  page.once('dialog', (dlg) => dlg.accept());
  await page.click('#endBtn');
  await page.waitForSelector('#over:not([hidden])', { timeout: 10000 });
  await sleep(300);
  await page.screenshot({ path: `${out}/8-game-over.png` });
  console.log(JSON.stringify({ t0, t1, dx_away: t1[0] - t0[0], z0, z1, dropped: z0 - z1, busyAfterDrop, gripAfter: grip, scoreBefore: before, scoreAfter: after, errors }, null, 1));
  await browser.close();
})().catch((e) => { console.error(e); process.exit(1); });
