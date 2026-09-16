import assert from "node:assert/strict";
import { mkdtemp, readFile, writeFile, unlink, rmdir } from "node:fs/promises";
import { tmpdir } from "node:os";
import { join } from "node:path";
import { fileURLToPath } from "node:url";
import { spawnSync } from "node:child_process";
import test from "node:test";

test("Graphviz renders, rejects invalid DOT, and preserves the input", async () => {
  const dir = await mkdtemp(join(tmpdir(), "hymeko-diagram-"));
  const input = join(dir, "source.dot");
  const output = join(dir, "diagram.svg");
  const script = fileURLToPath(new URL("render.mjs", import.meta.url));
  const run = (...args) => spawnSync(process.execPath, [script, ...args], { encoding: "utf8" });
  try {
    const source = 'digraph { n1 [id="decl-1", label="árvíz <frame>"]; n2 [id="decl-2"]; n1 -> n2 [id="incidence-0-0"]; }';
    await writeFile(input, source);
    assert.equal(run(input, output).status, 0);
    const svg = await readFile(output, "utf8");
    assert.match(svg, /<svg\b/);
    assert.match(svg, /árvíz &lt;frame&gt;/);
    assert.match(svg, /id="decl(?:-|&#45;)1"/);
    assert.equal(await readFile(input, "utf8"), source);
    assert.notEqual(run(input, input).status, 0);
    assert.equal(await readFile(input, "utf8"), source);
    await writeFile(input, "digraph { broken -> }");
    assert.notEqual(run(input, output).status, 0);
    assert.equal(await readFile(output, "utf8"), svg, "failure preserves the previous output");
  } finally {
    for (const path of [input, output]) await unlink(path).catch(error => { if (error.code !== "ENOENT") throw error; });
    await rmdir(dir);
  }
});
