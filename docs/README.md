# GSoC 2025: SITL AI Reinforcement Learning Concept Script
> **A Lua script that lets ArduPilot learn to tune itself, live, within SITL.**

**Mentors:** Nate Mailhot & Sanket Sharma&nbsp;&nbsp;|&nbsp;&nbsp;**Timeframe:** May → Sep 2025&nbsp;&nbsp;|&nbsp;&nbsp;**Proposal:** [PDF](https://summerofcode.withgoogle.com/media/user/2dbb09c4e712/proposal/gAAAAABoQLXhbncoQBtm8iphXPnXxy9u1VmpHHlJ4-Y7h8Myfb4XoS72AbFJ9QaXTHl-ZtOO36kkYyRNiD6PsO-woyzxx-VnAjBMh-gjEHC7Xm-IIP-8eZU=.pdf)

## Introduction
Hello Ardupilot community! I'm Bruno Andreoni Sarmento, a 4th year student at the Polytechnic School of the University of São Paulo (USP), pursuing a bachelor’s degree in Electrical Engineering. For the past two years I’ve been a member of USP’s autonomous-drone team, where I’ve worked with many open-source tools, ArduPilot being our core flight-control framework.
I've been inspired by other team members that took part in GSoC project, and it was a dream come true being selected to this awesome project!

## Project Proposal

Autonomous-drone parameter tuning still relies heavily on manual PID sweeps.
The goal of this project is to embed a lightweight reinforcement-learning layer (written in Lua and running natively inside ArduPilot SITL) that:
- Segments flights into reproducible episodes,
- Adjusts key parameters such as velocity and attitude gains online,
- Learns from reward signals like tracking error and settling time.

By the end of GSoC 2025 the community will have: a generic episode-reset API for SITL, a plug-and-play RL script with Q-learning/SARSA, and step-by-step docs so others can extend it.

## Architecture sketch

## Code sketch

## Timeline

## Call for feedback

