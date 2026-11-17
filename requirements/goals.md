# ferroforge-stm32's Goals

Every requirement derives from one of these, or from another requirement.

1. **G1 - Reusable hardware ports.** Each hardware port is a FerroForge task group
   that any firmware on any STM32 FerroHAL supports can select without editing it.
   Chip differences stay in FerroHAL; a group names FerroHAL's chip-neutral types.
2. **G2 - Described for generation.** Every group states its contract in its own
   declaration: the hardware slots it is built from by role, which member serves
   which slot's vector, the resources it leaves to the firmware with their types,
   and its constructor. A host tool reads it from source and selects the group
   without knowing which group it is. Groups and constructors also build for the
   host.
3. **G3 - Grounded in hardware.** Every hardware fact a port or its bench relies on
   cites ST's documentation (reference manual, datasheet, errata), and every port is
   proven on hardware on the rig, not by host checks alone.
4. **G4 - Requirement tracing and verification.** Each file states the requirements
   it fulfils and is hashed, and evidence records the hash (reqtool). After a first
   pass, only what changed is retested.
5. **G5 - No unsafe code of its own.** A chip's unsafe belongs to FerroHAL; this
   crate forbids `unsafe`.
