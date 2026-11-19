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

## The ADC port

`adc_event_port` is an event port that carries data (D27). One member owns
FerroHAL's `AdcScan` as its local resource; the firmware never names or locks
the converter.

- **A kick.** Any task holding the `Kick` handle (`Copy`) asks for a sequence:
  the kick adds one to a request count in the port's static memory
  (`KickMemory`) and pends the member through the function FerroForge
  generates for the selection (`pends`). The member starts the sequence.
- **The end.** When the sequence ends, the member spawns the firmware's task
  with a `Sequence` by value: up to 16 samples in channel order, the time the
  sequence started, its outcome, the sequences the task did not take
  (`missed`, and `missed_failed` of those) and the kicks that started nothing
  (`late_kicks`). A `Sequence` is 56 bytes, whatever the channel count.
- **The time** is the start's, read from the firmware's monotonic in the pass
  that started the sequence, because the samples' age counts from it.

The samples travel with the wake, so a later sequence cannot overwrite them
before the task reads them: the lost-result window of the shared-scan shape
is closed. That shape, `adc_scan`, is retired.

Nothing is lost silently (D8, D16):

- A kick while a sequence runs starts nothing; it is counted and carried by
  the next sequence the task takes. Kicks queued before the member runs start
  one sequence; the others are counted.
- A sequence that finds the task queued or running is dropped and counted.
- A start the driver refuses goes to the task as a sequence with no samples
  and the refusal as its outcome.

The constructor checks the sequence against the sample buffer and
`MAX_CHANNELS` before it opens anything, and hands back everything it took
when it refuses.

What the port cannot state: that a channel's pin is analog (DEBT D5). A
software kick moves by the run of any higher-priority task; a start triggered
by a timer in hardware, for sampling at a fixed rate, would be a FerroHAL
feature, and is not built.
