# Ten-method structural identification corpus

These ten **manually authored macro descriptions** make state/operator dependencies
comparable through the existing HyMeKo compiler and Query interface. They are not
ten executable optimizers, independently reviewed paper reconstructions, or a
performance ranking. The existing detailed DE/GA contracts remain authoritative for
their respective executable research profiles.

| File | Description | Source used for the macro mapping |
| --- | --- | --- |
| [de.hymeko](de.hymeko) | DE/rand/1/bin, bounded deferred reference | [Existing exact profile](../de_rand_1_bin.hymeko); [Storn–Price paper](https://doi.org/10.1023/A:1008202821328) |
| [ga.hymeko](ga.hymeko) | Tournament-2, uniform crossover, reset, stable mu+lambda | [Existing exact research profile](../ga_tournament_uniform_reset.hymeko) |
| [pso.hymeko](pso.hymeko) | Position, velocity, personal and global memory | [Kennedy–Eberhart, 1995](https://doi.org/10.1109/ICNN.1995.488968) |
| [hho.hymeko](hho.hymeko) | Energy-gated displacement and rapid dives | [Original paper](https://doi.org/10.1016/j.future.2019.02.028); [pinned author code](https://github.com/aliasgharheidaricom/Harris-Hawks-Optimization-Algorithm-and-Applications/blob/e5d854f64e0af5207e33c25dc196629b00d00d25/HHO.m) |
| [gwo.hymeko](gwo.hymeko) | Three ranked leaders, distance proposals and averaging | [Original paper, equations 3.1–3.7](https://doi.org/10.1016/j.advengsoft.2013.12.007) |
| [woa.hymeko](woa.hymeko) | Random/best-reference and spiral branches | [Original paper](https://doi.org/10.1016/j.advengsoft.2016.01.008); [author code distribution](https://seyedalimirjalili.com/woa) |
| [ba.hymeko](ba.hymeko) | Frequency/velocity, local perturbation, acceptance/adaptation | [Original preprint, section 3](https://arxiv.org/abs/1004.4170v1) |
| [fa.hymeko](fa.hymeko) | Pairwise attraction weighted by distance, perturbation | [Original preprint, sections 3.2–3.4](https://arxiv.org/abs/1003.1466v1) |
| [cs.hymeko](cs.hymeko) | Levy proposals, replacement, abandonment/restart | [Original preprint, section 3](https://arxiv.org/abs/1003.1594v1) |
| [abc.hymeko](abc.hymeko) | Neighbour proposals, quality selection and stagnation restart | [Author technical report TR06, section III](https://abc.erciyes.edu.tr/pub/tr06_2005.pdf) |

The author's WOA download was checked at retrieval: `WOA/WOA.m` SHA256
`baea13d019a18712465af9e72d5f374a75b502051e5e0a764c2b72e6d5569baa`.
The HHO source pin and the paper/author-code distinction are retained explicitly.
The Bat descriptor records unresolved source semantics. The ABC descriptor leaves
the numeric neighbourhood unspecified where TR06 does not fix it. All descriptions
have a declared abstraction boundary in `notes`; none silently resolves these gaps
into an executable reference.

## Representation and projection

Each file is a self-contained Core input. `method` contains typed states and typed
operator hyperedges; every operator has one neutral, unweighted read hyperarc and
one write hyperarc. Input/output meaning comes from the exact role types, not the
incidence sign. `space`, `category`, `family`, `guard`, and `schedule` use finite
generic vocabularies. `stage` is a unique ordinal in the declared macro ordering.
Rule text is retained for review, without an expression interpreter.

`hymeko_formats::metaheuristic::catalog::project(&CompiledModel)` reuses the existing
contract's Query checks, role admission and canonical identities. It rejects mixed
or missing roles, nonlocal/untyped/duplicate dependencies, wrong declaration kinds,
unrepresented fields, invalid stages and unknown categorical values. The standard
128 KiB source and 4096 declaration limits remain; each view admits at most 64
states and 64 operators.

Two feature multisets are exported:

- **Support:** each operator's category and read/write state-space multisets, plus
  each state's space and incident read/write operator categories. Shared state is
  therefore distinguishable from two separate states with identical local degrees.
- **Mechanisms:** replace categories with `(category, family, guard)` triples and
  add the declared schedule and consecutive macro-stage pairs.

Names, titles, source citations, canonical paths, raw IDs and Core hashes never
enter either feature vector. Core's digest remains provenance of the authored
document. Neither projection is a canonical graph hash or a complete isomorphism
test. Their multiset similarity is `sum(min(counts)) / sum(max(counts))`.

Exact equality identifies a **projected reference signature**. Multiple matches are
ambiguous; no match is unknown. Nearest neighbours are reported separately and are
never silently promoted to an exact identification. The chosen macro granularity
and vocabulary affect similarities; operator counts are not cost estimates.

## Running the test

Build the existing executable, then export one descriptor:

```sh
cargo build --locked --offline -p hymeko_formats --bin metaheuristic_contract
target/debug/metaheuristic_contract --structural \
  hymeko_formats/data/metaheuristics/catalog/hho.hymeko /path/to/NEW_EXPORT
```

The same command preserves canonical roundtrip verification and fresh-directory
protection. It writes `structure.json`, `contract.hymeko` and `contract.dot`.
Without `--structural`, the existing exact DE/GA export stays unchanged.

Run the research consumer from the Hystl repository:

```sh
python3 research/metaheuristics-pilot/identification_study.py  # preview
python3 research/metaheuristics-pilot/identification_study.py --run \
  --projector /path/to/transformation/target/debug/metaheuristic_contract \
  --corpus /path/to/transformation/hymeko_formats/data/metaheuristics/catalog \
  --output /path/to/NEW_IDENTIFICATION
```

It reuses the existing bounded command runner and artifact writer. No optimizer,
physics engine, LLM or new parser is invoked. There are 81 bounded projection
commands: ten references, 50 metamorphic observations, ten bad-role inputs, ten
mechanism changes and one arithmetic counterexample. Results include individual
verdicts, leave-one-method-out and duplicate-reference controls, source/binary
hashes, command receipts, JSON/CSV matrices and PNG/SVG heatmaps.

The 50 observations are deterministic transformations of these ten authored inputs,
not an independent test corpus or 50 optimizer runs. In particular, the DE
arithmetic counterexample changes the rule and Core digest while preserving both
feature multisets. This is a deliberate demonstration that signature equality does
not prove algorithm equivalence. Runtime identification and optimizer-performance
benchmarks require additional operational models and actual execution traces.

Owner checks:

```sh
cargo test --locked --offline -p hymeko_formats --test metaheuristic_catalog
python3 -m unittest discover -s research/metaheuristics-pilot \
  -p test_identification_study.py
```
