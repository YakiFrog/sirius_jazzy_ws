# Graphify code index

This workspace uses Graphify to give coding agents a compact, queryable map of
ROS packages and their cross-file relationships. The generated
`graphify-out/` directory is local and is not committed.

## Bootstrap

```bash
pipx install graphifyy==0.9.34
graphify codex install
graphify extract . --code-only --no-cluster
```

`--code-only` performs deterministic local AST extraction and does not send
source code to an external LLM. Run `graphify update .` after code changes.

## Agent queries

```bash
graphify query "How do BLE remote commands reach teleoperation?" --budget 1200
graphify explain "SiriusBleGateway"
graphify path "._on_remote_command()" ".handle_manual_teleop_instruction()"
```

The graph narrows repository exploration, but source files, launch files, ROS
runtime introspection, and tests remain the authority for behavior and
safety-critical changes.

Because the workspace also contains large upstream projects such as Nav2 and
RTAB-Map, prefer exact Sirius class or method names and set a query budget.
Broad natural-language queries can otherwise return structurally related but
operationally irrelevant dependency nodes.
