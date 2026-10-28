"""interfaces: different ways to make the OpenYAM arm move, all as armctl programs.

Every interface is a client of the armctl daemon (``ws://127.0.0.1:8091/armctl``), so the
lease, the envelope gate, the deadman and the E-STOP apply to all of them. See README.md.
"""
