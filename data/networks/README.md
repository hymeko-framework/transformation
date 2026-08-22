# network descriptions

A domain profile for computer networks, and a worked example.

| File | What it is |
|---|---|
| `meta_network.hymeko` | the vocabulary: devices, interfaces, media, protocols, services, zones, and the connectivity / policy / routing hyperedges |
| `office_lan.hymeko` | a small office network described end to end against that profile |

## Why a hypergraph fits this domain

A broadcast segment does not relate two interfaces — it relates every
interface attached to it, at once. Modelling a switched LAN as a bag of
pairwise cables throws away the very thing that matters, that those interfaces
share a broadcast domain, and then has to reconstruct it. Here the segment is
one edge:

```
@office_segment: nw.segment {
    (+ gw.lan, ~ sw1.uplink, ~ pc1.eth0, ~ pc2.eth0, ~ fileserver.eth0);
}
```

Compiled, that is a single 5-ary hyperedge, not four links.

Firewall rules are ternary for the same reason — a rule is *source*,
*destination* and *service* together, and splitting it loses the binding:

```
@admin_ssh: nw.allow {
    (+ pc1.eth0, - fileserver.eth0, ~ svc.ssh);
}
```

## Signed incidence

The profile uses one discipline everywhere, so a query can read the sign
without knowing the edge kind:

| sign | means |
|---|---|
| `+` | the serving / upstream / source side |
| `-` | the served / downstream / destination side |
| `~` | a peer, or a qualifier that is neither end |

So a segment is `(+ gateway_port, ~ peers…)`, a point-to-point run is
`(+ a_end, - z_end)`, a route is `(+ next_hop, - destination_subnet)`, and a
rule is `(+ source, - destination, ~ service)`.

## Querying it

The base hyperedges — `link_definition`, `policy_definition`,
`routing_definition` — exist so a query can ask for a whole category at once
rather than enumerating concrete kinds:

```hymeko
topology {}
context
{
    @connectivity: link_definition {}
    @rules:        policy_definition {}
    @routes:       routing_definition {}
    hosts:  device {}
    ports:  interface {}
}
```

## What the example compiles to

`office_lan.hymeko` yields 22 hyperedges, including:

```
office_segment   + gw.lan  ~ sw1.uplink  ~ pc1.eth0  ~ pc2.eth0  ~ fileserver.eth0
office_wifi      + ap1.radio  ~ laptop.wlan0
dmz_segment      + gw.dmz_port  ~ webserver.eth0
ap_backhaul      + sw1.p3  - ap1.wired
default_route    + fw.inside  - office_net
admin_ssh        + pc1.eth0  - fileserver.eth0  ~ ssh
```

## Not done yet

There is no `transforms/networks_*` directory, so nothing generates a
topology file from this. The graph preview in the VS Code plugin visualises it
already, since it works on any compiled description. A DOT or Mermaid
transformation would be a queries + template pair like the existing ones.

Nothing enforces arity: `@p2p_link` is documented as exactly two ends and the
grammar cannot say so. A query that counts incidence per edge would catch a
violation.
