# hdl-mini-pops

A dual-Arduino drum machine project based on the **Korg Mini Pops** emulation.

This project is a fork/evolution of the [Jupertronic Perculator](https://github.com/JuperTronic/Jupertronic_Perculator_Mini_Pops), which itself is based on Jan Ostman's O2 Mini Pops and Mark Dammer's "The Wee O3".

## Overview

The system uses two Arduino boards working in tandem:

1.  **Sound Board (`mini-pops-board`)**: Handles the audio generation, sequencing, and pattern logic.
2.  **Display Board (`display-board`)**: Handles the OLED display, user interface (knobs/buttons), and visual feedback.

## Key Features

-   **Classic Analog Drum Sounds**: 8-bit PWM samples emulating the Korg Mini Pops.
-   **Decoupled Pitch & Tempo**: Custom firmware modification allows you to change the **Pitch** (sample playback rate) independently of the **Tempo**. This allows for unique sound design possibilities not present in the original code.
-   **OLED Display**: Shows the current Pattern Name, Pattern Number, and BPM.
-   **16 Rhythm Patterns**: Selectable via potentiometer.

## Project Structure

-   `mini-pops-board/`: Firmware for the audio generation board.
    -   `mini-pops-board.ino`: Main firmware file.
    -   `TheWeeO3_data.h`: Audio sample data and pattern definitions.
-   `display-board/`: Firmware for the UI board.
    -   `display-board.ino`: Main firmware file (requires `ss_oled` library).

## Technical Details

### Pitch/Tempo Decoupling
In the original design, the sequencer logic was tied to the audio sample interrupt. This meant that lowering the pitch (slowing down the sample rate) would also slow down the tempo.

This version decouples them by:
-   Running the Audio Engine on **Timer 1** (variable frequency for pitch).
-   Running the Sequencer on **Timer 0** (fixed 1kHz frequency for stable tempo).

### Wiring
(Refer to the original [Jupertronic Perculator](https://github.com/JuperTronic/Jupertronic_Perculator_Mini_Pops) documentation for base wiring diagrams).

-   **Board 1 (Sound)** and **Board 2 (Display)** share a common Ground.
-   **Pattern Potentiometer**: Connected to Analog Inputs on *both* boards so they stay in sync.
-   **Start/Stop & Play State**: Digital signals passed between boards to sync the display with the sequencer state.

## Credits

-   **Jan Ostman**: Original O2 Mini Pops DSP code.
-   **Mark Dammer**: "The Wee O3" expansion.
-   **Jupertronic**: Perculator OLED integration and beat patterns.
-   **Bloghoskins**: Documentation and PCB designs.
