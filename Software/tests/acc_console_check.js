#!/usr/bin/env node
/* Drives the console's per-joint ACCELERATION path in headless Chrome.
 *
 *     node Software/tests/acc_console_check.js [path/to/arm-console.html]
 *     CHROME=/path/to/chrome node Software/tests/acc_console_check.js
 *
 * WHY THIS EXISTS
 *     Reading a console diff is not evidence. Two real crashes shipped past a
 *     careful read in this project and were caught only by running the page. So
 *     this loads the REAL arm-console.html, stubs nothing but the transport
 *     (httpJson) and send(), and exercises the acceleration path end to end
 *     against the REAL joint-limits.csv.
 *
 * WHAT IT PROVES
 *     1. joint-limits.csv's optional max_deg_per_sec2 column reaches J[].acc,
 *        and a joint with no value keeps the firmware default.
 *     2. The smoothness slider renders, carries the loaded value, and spans the
 *        same 5-250 the firmware accepts.
 *     3. Dragging it paints the readout and sends NOTHING; releasing it sends
 *        exactly one "ACC <j> <v>". A command per pixel of drag would flood a
 *        link whose whole design assumes one command in flight.
 *     4. Connecting pushes an ACC for every joint, after that joint's SPD.
 *
 * TWO TRAPS IT AVOIDS, both already documented by selftest.sh:
 *     Chrome's EXIT CODE is not the gate - a page that throws before rendering
 *     still exits 0. And grepping the whole DOM is not the gate either, because
 *     --dump-dom includes the driver source, which contains the literal strings
 *     ACC_PASS and ACC_FAIL. The <pre> verdict is extracted FIRST and only its
 *     contents are judged.
 *
 * No board, no bridge, no servo. This runs against file://.
 */
"use strict";

const fs = require("fs");
const os = require("os");
const path = require("path");
const { execFileSync } = require("child_process");

const REPO = path.resolve(__dirname, "..", "..");
const PAGE = process.argv[2] || path.join(REPO, "Software", "arm-console", "arm-console.html");
const LIMITS = path.join(REPO, "Software", "arm-console", "joint-limits.csv");

function findChrome() {
  if (process.env.CHROME) return process.env.CHROME;
  const candidates = [
    "C:/Program Files/Google/Chrome/Application/chrome.exe",
    "C:/Program Files (x86)/Google/Chrome/Application/chrome.exe",
    "/Applications/Google Chrome.app/Contents/MacOS/Google Chrome",
    "/usr/bin/google-chrome",
    "/usr/bin/chromium",
    "/usr/bin/chromium-browser",
  ];
  for (const c of candidates) if (fs.existsSync(c)) return c;
  return null;
}

const chrome = findChrome();
if (!chrome) {
  console.log("HARNESS ERROR: no Chrome found. Set CHROME=/path/to/chrome and re-run.");
  process.exit(2);
}

let html = fs.readFileSync(PAGE, "utf8");
/* core.autocrlf=true and there is no .gitattributes, so the working copy is
   CRLF. Normalise before embedding or the CSV parser sees a stray \r on the last
   field of every row - this exact thing has produced a false failure here before. */
const csv = fs.readFileSync(LIMITS, "utf8").replace(/\r\n/g, "\n");

const driver = `
<pre id="ACCVERDICT">PENDING</pre>
<script>
(function(){
  var lines = [], failed = 0;
  function say(s){ lines.push(String(s)); }
  function ck(name, got, want){
    var ok = JSON.stringify(got) === JSON.stringify(want);
    if (!ok) failed++;
    say((ok ? "  PASS  " : "  FAIL  ") + name +
        (ok ? "" : "   got=" + JSON.stringify(got) + " want=" + JSON.stringify(want)));
  }
  function flush(){
    document.getElementById("ACCVERDICT").textContent =
      (failed ? "ACC_FAIL (" + failed + ")" : "ACC_PASS") + "\\n" + lines.join("\\n");
  }

  try {
    USE_BRIDGE = true;
    var CSV = ${JSON.stringify(csv)};
    var sent = [];

    httpJson = function(method, p){
      if (String(p) === "/limits") return Promise.resolve({ ok:true, csv:CSV });
      if (String(p) === "/poses")  return Promise.resolve({ ok:true, csv:"pose_name,j0_deg\\n" });
      return Promise.reject(new Error("harness: unexpected " + method + " " + p));
    };
    send = function(line){ sent.push(String(line)); return Promise.resolve("OK"); };

    autoLoadLimits().then(function(){
      /* Read the expectations out of the CSV rather than restating them, so this
         keeps testing the WIRING when somebody retunes the wrist - which is the
         whole point of the slider and is expected to happen. */
      var want = {};
      CSV.split("\\n").forEach(function(l){
        if (!l || l.charAt(0) === "#" || l.indexOf("joint_id,") === 0) return;
        var f = l.split(",");
        if (f.length < 12) return;
        want[parseInt(f[0], 10)] = { dps: parseInt(f[6], 10), acc: parseInt(f[11], 10) };
      });

      Object.keys(want).forEach(function(k){
        var id = parseInt(k, 10);
        ck("J" + id + " acc comes from the file", J[id].acc, want[id].acc);
        ck("J" + id + " dps comes from the file", J[id].dps, want[id].dps);
      });

      var probe = 4;   // wrist pitch: the joint the gentle values exist for
      ck("the smoothness slider exists on J" + probe, !!J[probe].dom.acc, true);
      ck("slider shows the loaded value",  J[probe].dom.acc.value, String(want[probe].acc));
      ck("readout shows the loaded value", J[probe].dom.accVal.textContent, want[probe].acc + " deg/s2");
      ck("slider spans what the firmware accepts",
         [J[probe].dom.acc.min, J[probe].dom.acc.max], ["5","250"]);

      connState = "on";
      sent.length = 0;
      J[probe].dom.acc.value = "35";
      J[probe].dom.acc.oninput();
      ck("dragging paints the readout", J[probe].dom.accVal.textContent, "35 deg/s2");
      ck("dragging alone sends nothing", sent.length, 0);

      J[probe].dom.acc.onchange();
      ck("releasing sends exactly one command", sent.length, 1);
      ck("...and it is the right one", sent[0], "ACC " + probe + " 35");
      ck("the joint record was updated", J[probe].acc, 35);

      sent.length = 0;
      return pushState().then(function(){
        var accs = sent.filter(function(l){ return l.indexOf("ACC ") === 0; });
        ck("connect pushes one ACC per joint", accs.length, Object.keys(want).length);
        var iSpd = sent.indexOf("SPD " + probe + " " + want[probe].dps);
        var iAcc = sent.indexOf("ACC " + probe + " 35");
        ck("ACC follows SPD for the same joint", iSpd >= 0 && iAcc > iSpd, true);
        flush();
      });
    }).catch(function(e){
      say("threw: " + (e && e.message ? e.message : e));
      failed++; flush();
    });
  } catch (e) {
    say("threw synchronously: " + (e && e.message ? e.message : e));
    failed++; flush();
  }
})();
</script>
`;

if (!/<\/script>\s*$/.test(html)) {
  console.log("HARNESS ERROR: " + PAGE + " does not end in </script>; splice point unverified.");
  process.exit(2);
}

const tmp = fs.mkdtempSync(path.join(os.tmpdir(), "accdrive-"));
const page = path.join(tmp, "acc-harness.html");
fs.writeFileSync(page, html + driver);

let dom = "";
try {
  dom = execFileSync(chrome, [
    "--headless=new", "--disable-gpu", "--no-sandbox",
    "--virtual-time-budget=8000",
    "--dump-dom", "file:///" + page.replace(/\\/g, "/"),
  ], { encoding: "utf8", maxBuffer: 64 * 1024 * 1024, stdio: ["ignore", "pipe", "ignore"] });
} catch (e) {
  dom = (e && e.stdout) || "";
}

const m = dom.match(/<pre id="ACCVERDICT">([\s\S]*?)<\/pre>/);
if (!m) {
  console.log("ACC_FAIL (the page never rendered a verdict - it threw before the driver ran)");
  process.exit(1);
}
const text = m[1]
  .replace(/&lt;/g, "<").replace(/&gt;/g, ">")
  .replace(/&quot;/g, '"').replace(/&amp;/g, "&");
console.log(text.trim());

fs.rmSync(tmp, { recursive: true, force: true });
process.exit(/^ACC_PASS/m.test(text.trim()) ? 0 : 1);
