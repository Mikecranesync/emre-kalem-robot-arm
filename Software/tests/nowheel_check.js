/* Proves the arm console's number boxes cannot be changed by the scroll wheel.
 *
 *   node Software/tests/nowheel_check.js Software/arm-console/arm-console.html
 *
 * Run automatically by Software/tests/selftest.sh when node is available.
 * No board, no bridge, no browser -- a fake input and a fake document are the
 * only things noWheel() touches.
 *
 * WHY THIS EXISTS. A focused <input type="number"> steps its own value when the
 * wheel turns over it, with no click, no keystroke and nothing on screen to say
 * it happened. J0's speed sat at DPS=1 for an afternoon that way: a three-second
 * jog moved three degrees, and a healthy joint read as dead. The same hazard on
 * the ENABLE dialog's adopt box is worse -- that box is focused the instant the
 * dialog opens, and its value is the pulse the firmware pre-loads before
 * attaching the servo, which is the mechanism that snaps a joint.
 *
 * THIS TEST READS THE SHIPPED FUNCTION OUT OF THE HTML. It is not a copy, so it
 * fails if the real noWheel() is edited, weakened, or deleted.
 */

const fs = require("fs");

const page = process.argv[2];
if (!page) {
  console.log("usage: node nowheel_check.js <path to arm-console.html>");
  process.exit(2);
}

/* LINE ENDINGS ARE NORMALISED BEFORE ANY MATCHING.
 * The patterns below anchor on "\n" (e.g. /\nfunction noWheel\(input\)\{/), and
 * git checks this repo out with CRLF on Windows (core.autocrlf=true, no
 * .gitattributes). Without this the very first pattern misses and the test
 * announces "the scroll-wheel guard is gone" -- a false negative on an
 * untouched file, which is worse than no test at all, because it reports a
 * safety guard as deleted when it is present and working. Verified: the commit
 * this line was added in fails identically on a CRLF copy of the PREVIOUS
 * console. Nothing here is testing line endings, so normalising costs nothing.
 */
const src = fs.readFileSync(page, "utf8").replace(/\r\n/g, "\n");

/* Every number box on a joint card must be handed to noWheel(). A new one added
 * without that call is the regression this half of the test catches. */
const CLASSES = ["js-adoptval", "js-target", "js-dps"];

const m = src.match(/\nfunction noWheel\(input\)\{[\s\S]*?\n\}\n/);
if (!m) {
  console.log("FAIL: noWheel() is not in " + page + " -- the scroll-wheel guard is gone.");
  process.exit(1);
}

let activeElement = null;
const fakeDocument = { get activeElement() { return activeElement; } };
const noWheel = new Function("document", m[0] + "; return noWheel;")(fakeDocument);

function makeInput() {
  const listeners = [];
  return {
    listeners,
    addEventListener(type, handler, opts) { listeners.push({ type, handler, opts }); },
    fireWheel() {
      let prevented = false;
      const ev = { preventDefault() { prevented = true; } };
      listeners.filter((l) => l.type === "wheel").forEach((l) => l.handler(ev));
      return prevented;
    },
  };
}

let fails = 0;
function check(name, got, want) {
  const ok = JSON.stringify(got) === JSON.stringify(want);
  if (!ok) fails++;
  console.log((ok ? "  PASS  " : "  FAIL  ") + name +
              (ok ? "" : "   got=" + JSON.stringify(got) + " want=" + JSON.stringify(want)));
}

/* --- every number box on the card is actually guarded ---------------------
 * Counting `<input type="number"` in the raw file does NOT work: the prose in
 * arm-console.html quotes that tag when it explains the hazard, and the comment
 * scored as a fourth box. So match only inputs that carry a js- handle, which is
 * the only kind the card wiring can reach, and check them against the wiring
 * rather than against a hardcoded number. */
const boxes = (src.match(/<input type="number" class="(js-[A-Za-z-]+)"/g) || [])
  .map((t) => t.match(/class="(js-[A-Za-z-]+)"/)[1]);

check("the card's number boxes are the three known ones", boxes.sort(), CLASSES.slice().sort());

/* Resolve each js- class to the j.dom property it is stored under, then prove
 * THAT property is the one handed to noWheel(). This is what catches a fourth
 * box being added later without a guard. */
boxes.forEach(function (cls) {
  const decl = src.match(new RegExp("(\\w+)\\s*:\\s*q\\(\"\\." + cls + "\"\\)"));
  if (!decl) { check('"' + cls + '" has a j.dom handle', false, true); return; }
  const prop = decl[1];
  check('"' + cls + '" (j.dom.' + prop + ') is handed to noWheel()',
        new RegExp("noWheel\\(\\s*j\\.dom\\." + prop + "\\s*\\)").test(src), true);
});

check("noWheel() is called exactly once per number box",
      (src.match(/noWheel\(\s*j\.dom\./g) || []).length, boxes.length);

/* --- and the guard does what it claims ------------------------------------ */
const a = makeInput();
noWheel(a);

check("registers exactly one wheel listener", a.listeners.filter((l) => l.type === "wheel").length, 1);
check("registers it non-passive -- a passive listener cannot preventDefault",
      a.listeners[0] && a.listeners[0].opts && a.listeners[0].opts.passive, false);

activeElement = a;
check("FOCUSED box: the wheel is blocked, so the value cannot step", a.fireWheel(), true);

activeElement = null;
check("UNFOCUSED box: the wheel passes through, so the page still scrolls", a.fireWheel(), false);

const b = makeInput();
noWheel(b);
activeElement = a;
check("another card's focused box does not block this one", b.fireWheel(), false);
activeElement = b;
check("...and each box blocks its own", b.fireWheel(), true);

console.log(fails === 0 ? "NOWHEEL_PASS" : "NOWHEEL_FAIL (" + fails + ")");
process.exit(fails === 0 ? 0 : 1);
