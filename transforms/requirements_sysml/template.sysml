// Trace projection; selected syntax/relations checked, not native SysML tool conformance.
// Authored recorded_outcome is evidence data, not execution of the verification case.
package {{config:robot_name}} {
{{#each requirements}}
    requirement def {{name}} {
{{#if field:text}}        doc /* {{field:text}} */
{{/if}}    }
    requirement hymeko_req_{{name}} : {{name}};
{{/each}}
{{#each blocks}}
    part def {{name}};
    part hymeko_part_{{name}} : {{name}};
{{/each}}
{{#each tests}}
    verification def {{name}} {
        attribute recorded_outcome : ScalarValues::Integer = {{field:outcome}};
    }
{{/each}}
{{#each satisfies_edges}}
    satisfy requirement {{name}} references hymeko_req_{{bind:-:0}} by hymeko_part_{{bind:+:0}};
{{/each}}
{{#each allocated_edges}}
    allocation {{name}} allocate hymeko_req_{{bind:+:0}} to hymeko_part_{{bind:-:0}};
{{/each}}
    // Named connection projection of derivation; not a native deriveReqt assertion.
{{#each derives_edges}}
    connection hymeko_derive_{{name}} connect hymeko_req_{{bind:+:0}} to hymeko_req_{{bind:-:0}};
{{/each}}
{{#each verifies_edges}}
    verification {{name}} : {{bind:+:0}} {
        objective { verify hymeko_req_{{bind:-:0}}; }
    }
{{/each}}
}
