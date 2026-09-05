# Transformation Core-frissítés és nullgyök-korrekció

Created-at: 2026-09-05 21:41 +02:00.

Tesztelt implementáció: `24cfb59440c91801459937c7dcf6f8149a7f7723`.
A `74b5e177` dependency-commit a Core hivatkozását két sorban
`ac1131b5490734b855021f6076753417e1337cd6`-ra frissítette; Parser
`284c490d` és Tensor `b1d869d2` megmaradt. A következő, külön commit a valódi
Core-fixture-ekkel aktivált ellenőrzésen talált nullgyökhibát javítja.

## Eredmény és reprodukció

**470 PASS, 9 ignored, 0 FAIL**, teljes workspace, a Core robotikai és
dinamikai mintáival. A négy új, fixture nélkül is futó regresszió része ennek
a 470 tesztnek. A kilenc kihagyást a nyers napló név szerint tartalmazza.

```sh
# Az ellenőrzött Core SDK-képen belül; a forrás a /consumer útvonalon.
ln -s /workspace/core /core
cargo test --workspace --release --locked --offline --no-fail-fast
```

SDK: `hymeko-core:1886f17a42d7a3430e4044129f81955615efd583`, Rust/Cargo
1.97.1. Amaterasu, hálózat nélkül, 16 GiB/no-swap/6 CPU. A függőségeket
előtte külön korlátozott konténer töltötte le, ideiglenes SSH-agenttel és
szigorú hostkulcs-ellenőrzéssel. A manifest URL-jei kanonikusak; nincs path-patch.
A locked metadata pontosan egy Core és Parser source-azonosságot mutat.
Teszt után a Git-munkafa tiszta. A teszteknek írniuk kell az ignorált
`generated/` és eredményfájlokba, ezért a saját ellenőrzési checkout írható.

A kezdeti, üres targettel futó fordítás/teszt 47,47 s, cgroup-csúcs
1 453 805 568 B volt, de fixture-hiány miatt nem volt sikeres validáció.
A korrekciót és az összes mintát használó végső futás meglévő target-cache
mellett 5,32 s, cgroup-csúcs 86 106 112 B; exit 0, OOM false. Ez egyszeri
ellenőrzési mérés, nem tiszta fordítási vagy runtime-sebességi összehasonlítás.

[Végső eredmény](dependency-refresh/verified-results.json),
[teljes napló](dependency-refresh/verified-tests.log),
[konténeradat](dependency-refresh/verified-container-final.json),
[dependency metadata](dependency-refresh/metadata.json).

## Megkülönböztetett hibák

1. A Core sibling-minták nélkül két `articulated` teszt csak 8/24 esetig jutott.
   A többi, korai `return`-t használó fixture-teszt látszólagos PASS-ja nem
   validáció. A SDK meglévő mintáinak elérhetővé tételével mind a 18 geometriai
   teszt átment. Az első, szintén minták nélkül indított 20 ismétlés érvénytelen;
   naplója `modal-repeat-invalid.log` néven megmaradt.
2. A valódi láncmodell 20/20 futásban hat helyett hét módot adott. A plusz
   frekvencia `1.8348687944195805e-44 Hz`. A 13 koordinátás, veszteségmentes
   rendszernek hat oszcilláló párja és egy nullgyöke van. Ideiglenes diagnosztika
   rögzítette a mátrix bitjeit, a nyers gyököket és a kiszámított konstans
   együtthatót: `constant_bits=0000000000000000`.
3. A már helyes modal kód teljes futásában a read-only mount három query
   fájlkiíró tesztet blokkolt. A saját munkakönyvtár írásának engedélyezése után
   a fenti teljes kapu sikeres; a termékforrás ezen újrafutáskor nem változott.

[20 valódi reprodukció](dependency-refresh/modal-fixtures-repeat.log),
[mért együttható és gyökök](dependency-refresh/modal-diagnostic.log),
[ideiglenes instrumentáció](dependency-refresh/modal-instrumentation.patch),
[javítás előtti négy regresszió](dependency-refresh/modal-red.log),
[read-only mount hibája és modal green](dependency-refresh/final-tests.log).
Az instrumentáció nem része a végleges forrásnak.

## A numerikus korrekció határa

A meglévő `analysis::eigenvalues` a kiszámított karakterisztikus polinom
pontos nulla záró együtthatóit választja le. Ha `p(λ)=λ^z q(λ)`, a meglévő
Durand–Kerner iteráció csak `q` gyökeit keresi, majd `z` pontos komplex nulla
kerül a teljes gyökhalmazba. Fokszám nulla esetén az iteráció kimarad.
A gyökök rendezése, trace-/determinánsellenőrzése és toleranciája megmarad.
Nemnulla együtthatót a korrekció nem kerekít nullára.

A négy regresszió a kanonikus függvényt hívja független, zárt alakú
`0, ±iω` elvárásokkal: nullmátrixok, nullgyök-multiplicitás, `ω=2^-40`,
kettőhatványos skálázás és koordinátapermutáció. Mind a négy hibázott a régi
kódon; a nullmátrix NaN-okat is adott. A javítás után minden eset átment,
az eredeti láncspektrum `1e-9` relatív hibahatárával együtt.

Ez a faktorizálás a **kiszámított polinomra** pontos. A meglévő együttható-
képzés kondicionálását és underflow-ját nem oldja meg; trace és determináns
nem teljes sajátérték-tanúsítvány. Nagy vagy rosszul kondicionált sűrű mátrixokra
nem állítunk új garanciát. A korrekció a Transformation tulajdonában van;
a Core forrása és a mechanizált distributed-admission bizonyítás változatlan.

Terv: a Core `docs/plans/2026-09-05-dependency-refresh/plan.tex` és fordított
PDF-je, a végrehajtás előtt frissítve a két elkülönített változásra.
Rollback: a saját ágon előbb `24cfb594`, majd szükség esetén `74b5e177` revertje.

## Discovery

```text
DISCOVERY LEDGER — Core-fogyasztók revíziófrissítése
  find: meglévő Graph és Transformation audit-munkafák, Cargo-manifestek és lockfile-ok
  grep: Core-referenciák a root Cargo.toml fájlokban; Graph külön benchmark-lockkal
  symbols: Graph meglévő SYMBOLS.md; Transformationben nincs index
  read: Graph CLAUDE.md, CORE.YAML, governance script; mindkét README és Cargo-manifest
  verdict: EXTEND a meglévő dependency-fájlok a közös, már lefordított dependency-refresh terv szerint
           NEW dátumozott consumer-jelentések és nyers ellenőrzési adatok; nincs új tesztkeretrendszer

DISCOVERY LEDGER — modal nullgyök regresszió
  find: meglévő hymeko_dynamics/src/analysis.rs és tests/many_body.rs
  grep: eigenvalues az egyetlen eigensolver-tulajdonos; külön analysis integration test nincs
  symbols: meglévő publikus eigenvalues API; új publikus API nem szükséges
  read: karakterisztikus polinom, Durand–Kerner, modes szűrő, chain closed-form oracle
  verdict: EXTEND a két meglévő fájl; csak privát analitikus tesztfixture és comparator
```

A `.gitattributes` keresése nem talált meglévő fájlt; az új bejegyzések kizárólag
az auditbizonyíték bájtjainak megőrzésére szolgálnak.
