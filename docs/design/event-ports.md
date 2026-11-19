# Event ports and the ADC port

An event port is a hardware task that serves one source's vector and spawns
the firmware's own task with the event's time and the count of events that
task did not take (D25 of the port design record). The spawn is bound at
compile time, in the selection. The ADC port is an event port that carries
data (D27). This chapter is how the three groups apply that shape; the
contract's syntax is in [port-contract.md](port-contract.md).

| Group | Source | The firmware's task gets |
| --- | --- | --- |
| `exti_line` | one EXTI line, through FerroHAL's `ExtiHandle` | `edge(at_us, missed)` |
| `timer_tick` | one timer's update, through FerroHAL's `Tick` | `tick(at_us, missed)` |

## One shape

Each member does four things, in this order:

1. Reads the firmware's monotonic. The time is read first so that it is the
   event's, not the scheduler's: a task at any priority gets the time its
   event happened (D7, D26).
2. Asks the source whether it fired, and clears it. A pass with nothing
   pending (another source on a shared vector) does nothing else.
3. Spawns the firmware's task with the time and the count.
4. Keeps the count: zero once an event reached the task, one more when the
   spawn found the task still queued or running.

So every event the member took is delivered or counted, and the count arrives
with the next event delivered (D8). Nothing in a member waits for the
consumer, and nothing panics (D16, D19).

`timer_tick`'s member asks its local resource only for
`Service<Event = bool>`, which FerroHAL's `Tick` implements. FerroHAL's
`ExtiHandle` does not implement `Service` at the pinned revision, so
`exti_line` names it directly.

## What a member cannot count

A source flag is one bit: an EXTI line's pending bit, a timer's `UIF`. An
event raised while the last one is still pending, because a higher-priority
task held the member off for longer than a period, sets a bit already set and
is lost without a trace. Only the member's own lateness loses events this
way; a slow consumer does not.

## Time

Every port stamps from the firmware's RTIC monotonic, as microseconds in a
`u64` (D26). A stamp is as fine as that monotonic: 1 ms with the benches'
SysTick at 1 kHz. A finer stamp is the firmware's choice of monotonic, and
needs a timer; it is not a limit of the port.

## Shared vectors

EXTI lines 5 to 9 share one vector and 10 to 15 another; some timers share an
update vector (TIM1 with TIM10 on the F401 and F407, TIM8 with TIM13 on the
H753). RTIC binds one handler a vector, so one selection serves one source.
Two sources on one vector need a group that serves both, which is left until a
board needs one (D26).

## Rate

The source's rate is the firmware's: it routes the EXTI line, or builds the
`Tick` at its rate, in `init`, and hands it to the group's constructor. The
group has no config.
