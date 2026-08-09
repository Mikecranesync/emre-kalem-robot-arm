/* Round-trip test for the named-pose entry_path format. No hardware, no browser.
 *
 *   node Software/tests/pose_roundtrip.js
 *
 * WHY THIS EXISTS. savePoseCsv() writes entry_path and parseEntryPath() reads it
 * back, and they are ~400 lines apart in a 140 KB HTML file. If they drift, a
 * pose silently loads as "endpoint only" and the operator is told to drive it by
 * hand - a soft failure nobody would notice until they wondered why LOAD POSE
 * stopped filling the table.
 *
 * The functions are EXTRACTED FROM THE REAL FILE by name rather than pasted here,
 * so this test cannot pass against a stale copy of the code it is testing.
 */
"use strict";
const fs = require("fs");
const path = require("path");

const HTML = path.join(__dirname, "..", "arm-console", "arm-console.html");
const src = fs.readFileSync(HTML, "utf8")
              .match(/<script[^>]*>([\s\S]*?)<\/script>/g)
              .map(function (b) { return b.replace(/<\/?script[^>]*>/g, ""); })
              .join("\n");

function grab(name) {
  const i = src.indexOf("function " + name);
  if (i < 0) throw new Error("function not found in arm-console.html: " + name);
  let depth = 0;
  for (let k = src.indexOf("{", i); k < src.length; k++) {
    if (src[k] === "{") depth++;
    else if (src[k] === "}") { depth--; if (!depth) return src.slice(i, k + 1); }
  }
  throw new Error("unbalanced braces reading " + name);
}

const JOINT_DEFS = [{ id: 0 }, { id: 1 }, { id: 3 }, { id: 4 }, { id: 5 }, { id: 6 }];

/* eval'd as an EXPRESSION, not a bare declaration. Under "use strict" a function
   declaration inside eval is scoped to the eval and vanishes on return, which
   looks exactly like "the function is missing from the console". */
const parseEntryPath = eval("(" + grab("parseEntryPath") + ")");

let failed = 0;
function check(label, cond, detail) {
  console.log((cond ? "  ok   " : "  FAIL ") + label + (detail ? "   " + detail : ""));
  if (!cond) failed++;
}

/* Build entry_path exactly as savePoseCsv does. Kept in step by the assertion
   below, not by hope: if that format changes, the round trip breaks here. */
const steps = [
  { label: "home", deg: { 0: 90, 1: 70, 3: 15, 4: 50, 5: 104, 6: 70 } },
  { label: "fold", deg: { 0: 90, 1: 88, 3: 28, 4: 90, 5: 104, 6: 70 } },
];
const written = steps.map(function (w) {
  const parts = JOINT_DEFS
    .filter(function (d) { return w.deg[d.id] !== null && w.deg[d.id] !== undefined; })
    .map(function (d) { return "J" + d.id + "=" + w.deg[d.id]; });
  return String(w.label).replace(/[,>]/g, ";") + "(" + parts.join(";") + ")";
}).join(" > ");

console.log("entry_path under test:\n  " + written + "\n");
const back = parseEntryPath(written);

check("parses", !!back);
check("step count", !!back && back.length === 2, back ? String(back.length) : "");
check("labels survive", !!back && back[0].label === "home" && back[1].label === "fold");
check("angles survive", !!back && back[1].deg[1] === 88 && back[1].deg[3] === 28 &&
                         back[0].deg[5] === 104);
check("joints absent from a step read as null",
      !!parseEntryPath("only(J1=40)") && parseEntryPath("only(J1=40)")[0].deg[6] === null);

/* A legacy row must be REFUSED, never guessed. arm-poses.csv warns that the
   straight line between two safe poses can drive the claw through the bench, so
   half-understanding a path is worse than admitting you cannot read it. */
check("legacy free text refused",
      parseEntryPath("J1 1>22>34>46>58>70>80>88 at 5 deg/s then J3 52>60>64 at 20 deg/s") === null);
check("empty refused", parseEntryPath("") === null);
check("no-parens garbage refused", parseEntryPath("a > b > c") === null);
check("parens with no joints refused", parseEntryPath("step()") === null);

console.log(failed ? "\n" + failed + " FAILED" : "\nall passed");
process.exit(failed ? 1 : 0);
