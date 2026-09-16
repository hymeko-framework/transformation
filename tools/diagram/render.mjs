import { readFile, writeFile } from "node:fs/promises";
import { resolve } from "node:path";
import { instance } from "@viz-js/viz";

const [input, output, ...extra] = process.argv.slice(2);
if (!input || !output || extra.length || resolve(input) === resolve(output)) {
  throw new Error("Usage: node render.mjs INPUT.dot OUTPUT.svg (distinct files)");
}
const viz = await instance();
const result = viz.render(await readFile(input, "utf8"), { format: "svg", engine: "dot" });
if (result.status !== "success") {
  throw new Error(result.errors.map(error => error.message).join("\n"));
}
for (const warning of result.errors) console.warn(warning.message);
await writeFile(output, result.output, "utf8");
console.log(`SVG written: ${output} (Graphviz ${viz.graphvizVersion})`);
