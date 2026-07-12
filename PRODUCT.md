# Product

## Register

product

## Platform

web

## Users

The teach-pendant serves a mixed audience around a single robot workcell. The primary user is a robotics integrator or automation engineer doing initial setup — jogging the arm, capturing reference points, and defining the named coordinate frames the cell will run against. The secondary user is a less-technical operator who later needs to re-teach or adjust a frame when the cell is moved, bumped, or reconfigured. Both are standing at or near the robot, working through a physical calibration task rather than reading or exploring; the screen is a means to move metal accurately. The interface has to carry the engineer through a precise multi-step procedure and still let the operator re-teach a frame without a manual open beside them.

## Product Purpose

teach-frames turns physical arm motion into named, persistent workcell coordinate frames — the software equivalent of a UR teach-pendant plane calibration, inside the Viam ecosystem. The user jogs the arm's TCP to reference points, captures them, and defines a frame by method (3-point, single point, TCP snapshot); the frame is computed, persisted back into the component's own config so it survives restarts, exposed through the PoseTracker API, and mirrored to the Viam visualizer as axes triads. Success is a low-friction session: someone can teach a correct frame with little instruction and few mistakes, and re-teaching later feels routine rather than risky.

## Positioning

The pendant makes it possible to define real, named, persistent coordinate frames by physically moving the arm — no hand-editing of frame config, no separate calibration rig — turning the workcell's geometry into something the robot can reason about.

## Brand Personality

Confident and responsive. Controls answer immediately, feedback is instant, and the live pose and joint readouts always reflect the true state of the arm — the tool feels direct and gets out of the way of the task. The voice is plain and operational: name the action, show the result, surface errors without drama. It should read as a focused instrument, not a consumer app and not a marketing surface.

## Anti-references

Not consumer or playful — no bright toy-like styling, gamification, or cute friendliness. Not a flashy marketing UI — no gradients-everywhere, hero moments, decorative motion, or SaaS-landing sparkle. Not cramped or cluttered — the density of controls and readouts must stay legible with real breathing room, never a wall of tiny buttons.

## Design Principles

Low-friction over feature-completeness. The mixed audience means the common path — jog, capture, define, verify — has to be obvious enough that an operator can follow it without training; depth is available but never in the way.

Honest, live state. This is a pendant driving physical hardware, so the readout must always reflect the arm's true pose and joints, feedback must be immediate, and the UI must never imply motion or a captured point that didn't happen.

Direct and responsive. Actions map to results with no ceremony; the interface answers the user faster than they can second-guess it.

Belongs in Viam, speaks for itself. Cohesive with the surrounding Viam app in palette and component vocabulary, but recognizable as its own focused teach-pendant surface.

Function over decoration. Every visual choice earns its place by serving the calibration task; nothing is there to impress.

## Accessibility & Inclusion

Target WCAG AA across the interface: sufficient contrast for body text, labels, and the numeric readouts; visible focus states; and controls that meet minimum touch-target sizing (the jog controls already use 44px targets).
