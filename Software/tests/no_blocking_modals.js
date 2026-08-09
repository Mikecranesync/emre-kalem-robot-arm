/*
 * no_blocking_modals.js
 *
 *   node Software/tests/no_blocking_modals.js Software/arm-console/arm-console.html
 *
 * Run automatically by Software/tests/selftest.sh when node is available.
 *
 * WHY THIS TEST EXISTS — it is not a style rule.
 *
 * alert(), confirm() and prompt() block the JavaScript event loop. Nothing else
 * runs while one is open: no timer, no promise, no fetch callback. The console
 * feeds the controller a PNG heartbeat every 250 ms (PING_MS) and the firmware
 * detaches every joint if it hears nothing for 4000 ms (WATCHDOG_MS). Four
 * seconds is how long it takes to read a dialog and type a word into it.
 *
 * So a modal is not a pause. It is an arm on the floor: the firmware answers
 * EVT WDOG, latchEstopUi() fires, every joint detaches, and a gravity-loaded
 * arm sags. On screen it reads as an e-stop that nobody pressed.
 *
 * Two shipped in the console before this test existed:
 *   - prompt() for the pose name in savePoseCsv
 *   - confirm() on DISCONNECT while joints were enabled and holding, which is
 *     the worse one: it fired ONLY when joints were live, and its whole purpose
 *     was to prevent an unsupported detach.
 *
 * Ask in the page instead. #confirmStrip is the pattern: same words, same
 * decision, timers still running.
 */

const fs = require("fs");

const page = process.argv[2];
if (!page) {
  console.log("usage: node no_blocking_modals.js <path to arm-console.html>");
  process.exit(2);
}

/* Line endings normalised: git checks this repo out CRLF on Windows
 * (core.autocrlf=true, no .gitattributes) and the line numbers below are
 * reported to a human. */
const src = fs.readFileSync(page, "utf8").replace(/\r\n/g, "\n");
const lines = src.split("\n");

/* STRUCTURAL, NOT A GREP FOR "window.prompt". Matches the bare call too
 * (prompt(...)), the window-qualified form, and the bracket form
 * window["prompt"] — the three ways this comes back. \b stops it firing on
 * promptUser, confirmAdopt, alertBox and similar honest names. */
const CALL   = /(?:\bwindow\s*\.\s*)?\b(alert|confirm|prompt)\s*\(/;
const BRACKET = /\bwindow\s*\[\s*["'](alert|confirm|prompt)["']\s*\]/;

/* The comments in the console explain this hazard by name, and the test file
 * list in selftest.sh mentions it too. Prose is not a call. */
function isProse(line) {
  const t = line.trim();
  return t.startsWith("*") || t.startsWith("//") || t.startsWith("/*") || t.startsWith("<!--");
}

const hits = [];
lines.forEach((line, i) => {
  if (isProse(line)) return;
  const m = CALL.exec(line) || BRACKET.exec(line);
  if (m) hits.push({ n: i + 1, name: m[1], text: line.trim() });
});

if (hits.length) {
  console.log("FAIL: " + hits.length + " blocking modal call(s) in " + page + ".");
  console.log("      A modal stops the heartbeat; the firmware detaches every joint after " +
              "4 s and the arm sags.");
  console.log("      Ask in the page instead — see #confirmStrip.");
  console.log("");
  hits.forEach((h) => {
    console.log("  line " + h.n + "  " + h.name + "()");
    console.log("      " + (h.text.length > 110 ? h.text.slice(0, 110) + " ..." : h.text));
  });
  process.exit(1);
}

console.log("  PASS  no alert() / confirm() / prompt() — nothing can stall the heartbeat");
console.log("MODALS_PASS");
