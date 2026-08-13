# RoboLLM · Documentation style guide

> **RoboLLM field guide** · Build → Observe → Measure → Learn<br>
> [Home](../README.md) · [Documentation](README.md) · [Roadmap](../ROADMAP.md) · [Architecture](ARCHITECTURE.md)

This guide keeps repository documentation recognizable, navigable, and honest.
It applies to first-party Markdown and SVG files; vendored upstream documents
retain their original voice and formatting.

## Document anatomy

1. Use `# RoboLLM · <subject>` for the page title.
2. Follow the title with the field-guide navigation strip.
3. Lead with purpose and current status before implementation detail.
4. Put the shortest working path before optional variants.
5. Keep commands copyable from the directory named by the page.
6. End runbooks with success signals and recovery guidance.

## Status vocabulary

| Label | Meaning |
|---|---|
| **Verified** | Reproducible evidence exists for the named environment. |
| **Code-ready** | Implementation and hardware-free tests exist; target toolchain evidence is still open. |
| **Bench-gated** | Real measurements, wiring, calibration, or physical acceptance are required. |
| **Planned** | Architecture or research direction exists, but the capability is not delivered. |

Never promote a simulation result to physical-hardware evidence. Always name
the environment behind a measured number.

## Voice and notation

- Prefer direct verbs: build, launch, inspect, measure, reject, recover.
- Use `RoboLLM` for the project and neutral “MCP client” language for protocol
  behavior; name Claude only where its client or packaging format matters.
- Use radians above the physical-arm driver and degrees only at the serial
  boundary.
- Use relative repository paths instead of machine-specific home directories.
- Keep the invariant visible: AI proposes structured goals; deterministic
  validation and firmware safety decide what reaches actuators.

## Visual language

Architecture SVGs use one shared palette:

| Token | Color | Use |
|---|---|---|
| Canvas | `#F8FAFC` | Diagram background and light-on-dark text |
| Ink | `#0F172A` | Titles and primary text |
| Primary | `#0F6CBD` | Implemented or verified components |
| Accent | `#38BDF8` | Top rule and small brand details |
| External | `#64748B` | People, devices, and external systems |
| Planned | `#E2E8F0` | Future capabilities, normally with a dashed stroke |
| Gated | `#D97706` | Partial capability or pending physical acceptance |

Every diagram uses the `RoboLLM · <subject>` title pattern, the shared accent
rule, and `Inter, Segoe UI, Arial, sans-serif` typography.

Diagrams must include an accessible `<title>` and `<desc>`, parse as XML, and
render without external fonts or assets.
