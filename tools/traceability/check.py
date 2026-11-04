"""Recheck retained trace artifacts against complete IR and hand-authored references.

This parses the documented emitted subset, not arbitrary SysML or a native SysML AST.
No compilation fallback, missing-view skip, timing loop or robot execution.
"""
from __future__ import annotations
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path
import argparse
import copy
import csv
import graphlib
import io
import json
import re

ROLES = {'satisfies': ('component', 'requirement'), 'allocated_to': ('requirement', 'component'),
         'derives': ('requirement', 'requirement'), 'verifies': ('test_case', 'requirement')}
NAME = r'([A-Za-z_][A-Za-z_0-9]*)'


class AuditError(ValueError):
    pass


@dataclass
class TraceInvariant:
    entities: dict = field(default_factory=dict)
    relations: dict = field(default_factory=dict)

    def entity(self, name, kind, text=None, outcome=None):
        if name in self.entities:
            raise AuditError('duplicate entity: ' + name)
        self.entities[name] = dict(kind=kind, text=text, outcome=outcome)

    def relation(self, name, kind, source, target):
        if name in self.relations:
            raise AuditError('duplicate relation identity: ' + name)
        self.relations[name] = dict(kind=kind, source=source, target=target)

    def record(self):
        for name, relation in self.relations.items():
            if relation['kind'] not in ROLES:
                raise AuditError('unknown relation kind: ' + name)
            for role, expected in zip(['source', 'target'], ROLES[relation['kind']]):
                if self.entities.get(relation[role], {}).get('kind') != expected:
                    raise AuditError('invalid relation role: ' + name)
        for name, entity in self.entities.items():
            if entity['kind'] == 'test_case' and entity['outcome'] not in [-1, 0, 1]:
                raise AuditError('invalid recorded outcome: ' + name)
        return dict(entities=self.entities, relations=self.relations)


class TraceView(ABC):
    @abstractmethod
    def extract(self, text: str) -> dict:
        pass


class SysmlView(TraceView):
    def extract(self, text):
        result = TraceInvariant()
        active = None
        usages = {'requirement': set(), 'component': set()}
        opened = closed = False
        for line in (line.strip() for line in text.splitlines()):
            if not line or line.startswith('//'):
                continue
            if closed:
                raise AuditError('text after SysML package')
            if not opened:
                if not re.fullmatch('package ' + NAME + r' \{', line):
                    raise AuditError('missing SysML package')
                opened = True
                continue
            if active:
                kind, name, value = active
                if line == '}':
                    if kind == 'requirement':
                        result.entity(name, kind, text=value)
                    elif kind == 'test_case' and value is not None:
                        result.entity(name, kind, outcome=value)
                    elif kind == 'verifies' and isinstance(value, tuple):
                        result.relation(name, kind, *value)
                    else:
                        raise AuditError('incomplete SysML declaration: ' + name)
                    active = None
                    continue
                match = re.fullmatch(r'doc /\* (.*?) \*/', line)
                if kind == 'requirement' and match and value is None:
                    active = (kind, name, match[1]); continue
                match = re.fullmatch(r'attribute recorded_outcome : ScalarValues::Integer = (-1|0|1);', line)
                if kind == 'test_case' and match and value is None:
                    active = (kind, name, int(match[1])); continue
                match = re.fullmatch(r'objective \{ verify hymeko_req_' + NAME + r'; \}', line)
                if kind == 'verifies' and match and isinstance(value, str):
                    active = (kind, name, (value, match[1])); continue
                raise AuditError('unrecognized SysML member: ' + line)
            if line == '}':
                closed = True; continue
            match = re.fullmatch('(requirement|verification) def ' + NAME + r' \{', line)
            if match:
                active = ('requirement' if match[1] == 'requirement' else 'test_case', match[2], None); continue
            match = re.fullmatch('part def ' + NAME + ';', line)
            if match:
                result.entity(match[1], 'component'); continue
            match = re.fullmatch('(requirement|part) hymeko_(req|part)_' + NAME + ' : ' + NAME + ';', line)
            if match:
                kind = 'requirement' if match[1] == 'requirement' else 'component'
                if match[3] != match[4] or match[2] != ('req' if kind == 'requirement' else 'part') or match[3] in usages[kind]:
                    raise AuditError('invalid SysML usage')
                usages[kind].add(match[3]); continue
            match = re.fullmatch('satisfy requirement ' + NAME + ' references hymeko_req_' + NAME + ' by hymeko_part_' + NAME + ';', line)
            if match:
                result.relation(match[1], 'satisfies', match[3], match[2]); continue
            match = re.fullmatch('allocation ' + NAME + ' allocate hymeko_req_' + NAME + ' to hymeko_part_' + NAME + ';', line)
            if match:
                result.relation(match[1], 'allocated_to', match[2], match[3]); continue
            match = re.fullmatch('connection hymeko_derive_' + NAME + ' connect hymeko_req_' + NAME + ' to hymeko_req_' + NAME + ';', line)
            if match:
                result.relation(match[1], 'derives', match[2], match[3]); continue
            match = re.fullmatch('verification ' + NAME + ' : ' + NAME + r' \{', line)
            if match:
                active = ('verifies', match[1], match[2]); continue
            raise AuditError('unrecognized SysML statement: ' + line)
        if not closed or active:
            raise AuditError('incomplete SysML output')
        for kind, names in usages.items():
            if names != {name for name, row in result.entities.items() if row['kind'] == kind}:
                raise AuditError('missing or extra SysML usages: ' + kind)
        return result.record()


class DotView(TraceView):
    @staticmethod
    def attributes(text):
        attrs = {}
        while text.strip():
            match = re.match(r'\s*([A-Za-z_]+)=("(?:\\.|[^"\\])*"|[A-Za-z_0-9]+)\s*(?:,\s*|$)', text)
            if not match or match[1] in attrs:
                raise AuditError('invalid or duplicate DOT attribute')
            attrs[match[1]] = json.loads(match[2]) if match[2].startswith('"') else match[2]
            text = text[match.end():]
        return attrs

    def extract(self, text):
        result = TraceInvariant(); opened = closed = False
        for line in (line.strip() for line in text.splitlines()):
            if not line or line.startswith('//'):
                continue
            if closed:
                raise AuditError('text after DOT graph')
            if not opened:
                if not re.fullmatch('digraph ' + NAME + r' \{', line):
                    raise AuditError('missing DOT graph')
                opened = True; continue
            if line == '}':
                closed = True; continue
            if line == 'rankdir=LR;' or re.fullmatch(r'(node|edge) \[.*\];', line):
                continue
            match = re.fullmatch('"' + NAME + '" -> "' + NAME + r'" \[(.*)\];', line)
            if match:
                attrs = self.attributes(match[3])
                kind = {'<<satisfy>>': 'satisfies', '<<allocate>>': 'allocated_to', '<<deriveReqt>>': 'derives', '<<verify>>': 'verifies'}.get(attrs.get('label'))
                if kind is None or 'id' not in attrs:
                    raise AuditError('missing DOT relation identity/type')
                result.relation(attrs['id'], kind, match[1], match[2]); continue
            match = re.fullmatch('"' + NAME + r'" \[(.*)\];', line)
            if match:
                attrs = self.attributes(match[2]); label = attrs.get('label', '')
                parsed = re.fullmatch(r'<<(requirement|block|test_case)>>\n' + re.escape(match[1]), label)
                if parsed is None:
                    raise AuditError('invalid DOT entity label')
                kind = 'component' if parsed[1] == 'block' else parsed[1]
                result.entity(match[1], kind, attrs.get('text') or None, int(attrs['outcome']) if 'outcome' in attrs else None); continue
            raise AuditError('unrecognized DOT statement: ' + line)
        if not closed:
            raise AuditError('incomplete DOT output')
        return result.record()


class CsvView(TraceView):
    def extract(self, text):
        reader = csv.DictReader(io.StringIO(text), strict=True)
        if reader.fieldnames != ['record', 'kind', 'name', 'source', 'target', 'text', 'outcome']:
            raise AuditError('invalid CSV schema')
        result = TraceInvariant()
        for row in reader:
            if None in row or any(value is None for value in row.values()):
                raise AuditError('invalid CSV row width')
            if row['record'] == 'entity' and not row['source'] and not row['target']:
                if row['kind'] not in ['requirement', 'component', 'test_case']:
                    raise AuditError('invalid CSV entity kind')
                result.entity(row['name'], row['kind'], row['text'] or None, int(row['outcome']) if row['outcome'] else None)
            elif row['record'] == 'relation' and not row['text'] and not row['outcome']:
                result.relation(row['name'], row['kind'], row['source'], row['target'])
            else:
                raise AuditError('invalid CSV record')
        return result.record()


VIEWS = {'sysml': SysmlView(), 'dot': DotView(), 'csv': CsvView()}


def coverage(record):
    entities, relations = record['entities'], list(record['relations'].values())
    rows = {}
    for name, entity in sorted(entities.items()):
        if entity['kind'] != 'requirement':
            continue
        realizers = sorted({r['source'] for r in relations if r['kind'] == 'satisfies' and r['target'] == name})
        allocated = sorted({r['target'] for r in relations if r['kind'] == 'allocated_to' and r['source'] == name})
        tests = sorted({r['source'] for r in relations if r['kind'] == 'verifies' and r['target'] == name})
        all_pass = bool(tests) and all(entities[test]['outcome'] == 1 for test in tests)
        rows[name] = dict(realizers=realizers, allocated=allocated, tests=tests, all_recorded_pass=all_pass,
                          complete_trace=bool(realizers) and all_pass,
                          failed=[test for test in tests if entities[test]['outcome'] == 0],
                          unknown=[test for test in tests if entities[test]['outcome'] == -1],
                          orphan=not any(name in [r['source'], r['target']] for r in relations))
    graph = {name: set() for name in rows}
    for relation in relations:
        if relation['kind'] == 'derives':
            graph[relation['source']].add(relation['target'])
    try:
        list(graphlib.TopologicalSorter(graph).static_order()); dag = True
    except graphlib.CycleError:
        dag = False
    return {'requirements': rows, 'derivation_acyclic': dag,
            'counts': {'total': len(rows), 'allocated': sum(bool(r['allocated']) for r in rows.values()),
                       'realized': sum(bool(r['realizers']) for r in rows.values()),
                       'test_linked': sum(bool(r['tests']) for r in rows.values()),
                       'all_recorded_pass': sum(r['all_recorded_pass'] for r in rows.values()),
                       'complete_trace': sum(r['complete_trace'] for r in rows.values()),
                       'orphan': sum(r['orphan'] for r in rows.values())}}


def compare(texts, reference):
    if set(texts) != set(VIEWS):
        raise AuditError('required views missing or unexpected')
    for extension, strategy in VIEWS.items():
        actual = strategy.extract(texts[extension])
        if actual != reference:
            raise AuditError(extension + ' differs from full reference')
    return coverage(reference)


def fault_controls(texts, reference):
    controls = []
    def detect(name, change):
        changed = dict(texts); change(changed)
        try:
            compare(changed, reference)
        except (AuditError, ValueError, KeyError):
            controls.append({'fault': name, 'detected': True}); return
        raise AuditError('undetected injected fault: ' + name)
    detect('missing_required_view', lambda v: v.pop('csv'))
    detect('empty_required_view', lambda v: v.__setitem__('sysml', ''))
    detect('same_wrong_target_in_all_views', lambda v: v.update({k: t.replace('R_stop', 'R_wrong') for k, t in v.items()}))
    detect('changed_requirement_text', lambda v: v.__setitem__('sysml', v['sysml'].replace('200 ms', '900 ms')))
    detect('changed_test_outcome', lambda v: v.__setitem__('dot', v['dot'].replace('outcome="0"', 'outcome="1"')))
    detect('removed_relation', lambda v: v.__setitem__('csv', '\n'.join(line for line in v['csv'].splitlines() if ',sat_stop_review,' not in line)))
    line = next(line for line in texts['csv'].splitlines() if ',sat_stop,' in line)
    detect('duplicate_relation_id', lambda v: v.__setitem__('csv', v['csv'] + '\n' + line + '\n'))
    detect('swapped_endpoint_roles', lambda v: v.__setitem__('dot', v['dot'].replace('"T_stop" -> "R_stop"', '"R_stop" -> "T_stop"')))
    detect('wrong_relation_type', lambda v: v.__setitem__('csv', v['csv'].replace('relation,verifies,verify_stop', 'relation,satisfies,verify_stop')))
    detect('missing_entity', lambda v: v.__setitem__('csv', '\n'.join(line for line in v['csv'].splitlines() if ',T_stop,' not in line)))
    detect('malformed_statement', lambda v: v.__setitem__('sysml', v['sysml'].replace('by hymeko_part_SafetyController;', 'BROKEN;')))
    return controls


def expected_for(name, baseline):
    result = copy.deepcopy(baseline)
    if name == 'value-after':
        result['entities']['T_cycle']['outcome'] = 1
    if name == 'structure-after':
        for key, kind, text, outcome in [('R_logging', 'requirement', 'Persist the audit log', None), ('Logger', 'component', None, None), ('T_logging', 'test_case', None, 1)]:
            result['entities'][key] = dict(kind=kind, text=text, outcome=outcome)
        for key, kind, source, target in [('sat_logging', 'satisfies', 'Logger', 'R_logging'), ('verify_logging', 'verifies', 'T_logging', 'R_logging'), ('alloc_logging', 'allocated_to', 'R_logging', 'Logger')]:
            result['relations'][key] = dict(kind=kind, source=source, target=target)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('outputs', type=Path)
    parser.add_argument('--expected', type=Path, default=Path(__file__).with_name('expected.json'))
    parser.add_argument('--result', type=Path, required=True)
    args = parser.parse_args()
    baseline = json.loads(args.expected.read_text('utf-8'))
    names = ['baseline', 'value-before', 'value-after', 'retained-value-before', 'structure-before', 'structure-after', 'retained-structure-before', 'reordered']
    results = []
    captured = {}
    for name in names:
        folder = args.outputs / name
        reference = json.loads((folder / 'reference.json').read_text('utf-8'))
        if reference != expected_for(name, baseline):
            raise AuditError('IR reference differs from hand reference: ' + name)
        texts = {extension: (folder / ('trace.' + extension)).read_text('utf-8') for extension in VIEWS}
        result = compare(texts, reference)
        with (folder / 'coverage.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream); writer.writerow(['requirement', 'allocated_to', 'realized_by', 'tests', 'all_recorded_pass', 'complete_trace', 'failed', 'unknown', 'orphan'])
            for requirement, row in result['requirements'].items():
                writer.writerow([requirement, ';'.join(row['allocated']), ';'.join(row['realizers']), ';'.join(row['tests']), row['all_recorded_pass'], row['complete_trace'], ';'.join(row['failed']), ';'.join(row['unknown']), row['orphan']])
        results.append(dict(case=name, full_reference_equal=True, views=3, **result))
        captured[name] = texts
    controls = fault_controls(captured['baseline'], baseline)
    mixed = dict(captured['value-after'], dot=captured['baseline']['dot'])
    try:
        compare(mixed, expected_for('value-after', baseline))
    except AuditError:
        controls.append({'fault': 'stale_view_after_publication', 'detected': True})
    else:
        raise AuditError('stale view was not detected')
    cov = coverage(baseline)
    assert cov['counts'] == dict(total=8, allocated=3, realized=6, test_linked=6, all_recorded_pass=4, complete_trace=4, orphan=1)
    assert not cov['requirements']['R_training']['realizers'] and cov['requirements']['R_training']['allocated']
    assert cov['requirements']['R_cycle']['failed'] == ['T_cycle']
    assert cov['requirements']['R_grip']['unknown'] == ['T_grip']
    mixed_ref = copy.deepcopy(baseline)
    mixed_ref['relations']['verify_stop_failed'] = dict(kind='verifies', source='T_cycle', target='R_stop')
    assert not coverage(mixed_ref)['requirements']['R_stop']['all_recorded_pass']
    cycle_ref = copy.deepcopy(baseline)
    cycle_ref['relations']['cycle'] = dict(kind='derives', source='R_stop', target='R_diagnostics')
    assert not coverage(cycle_ref)['derivation_acyclic']
    output = dict(all_views_faithful=True, scenarios=results, output_fault_controls=controls,
                  coverage_controls=['allocation_not_realization', 'failed_test', 'unknown_test', 'orphan', 'mixed_pass_and_fail', 'derivation_cycle'],
                  scope='Synthetic recorded outcomes; subset extractors, no native SysML acceptance or physical tests')
    args.result.write_text(json.dumps(output, indent=2) + '\n', encoding='utf-8')
    print(json.dumps({'scenarios': len(results), 'required_view_checks': len(results)*3, 'output_faults_detected': len(controls), 'baseline_coverage': cov['counts']}, indent=2))


if __name__ == '__main__':
    main()
