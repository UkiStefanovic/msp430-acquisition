# MSP430 Embedded Data Acquisition

This project is part of an embedded systems course focusing on the MSP430F5529LP platform. It implements real-time data acquisition from multiple channels using the ADC. The collected data is transmitted over UART to the PC.

## Overview

- **Platform:** MSP430F5529LP + ETF Shield
- **Language:** C
- **Main File:** main.c

## Features

- Data acquisition from multiple channels
- UART communication for data transmission
- Button-controlled start and stop of data acquisition

## Getting Started

### Prerequisites

- Code Composer Studio
- MSP-EXP430F5529LP + ETF Shield

### Installation

1. Clone the repository.
2. Open the project in Code Composer Studio.
3. Connect your MSP430 LaunchPad and build/upload the code.

## Usage

- Press SW1 button to start data acquisition.
- Press SW2 button to stop data acquisition.
