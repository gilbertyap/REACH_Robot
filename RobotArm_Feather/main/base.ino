// Power reduction formula for adjusting the turning side's speed
int turnSpeedReduction(uint8_t turnValue) {
  return exp((-1 * turnValue) / (2 * sqrt(256)));
}

// Sets the power of each wheel, and what direction
// TODO NEED TURN TOLERANCES
void setBaseMotors(uint8_t baseBuf[]) {
  // Determine the direction of turning
  int8_t turnDirection;
  if(baseBuf[0] > 128) {
    turnDirection = 1; // right turn
  } else if (baseBuf[0] < 128) {
    turnDirection = -1; // left turn
  } else {
    turnDirection = 0; // no turn
  }

  // Determine forward or backwards
  int8_t speedDirection;
  if(baseBuf[1] >= 128) {
    speedDirection = 1; // forward
  } else {
    speedDirection = -1; // backwards
  }

  // Adjust the value to be from [0,256]
  int8_t turnRatio = baseBuf[0] >= 128 ? 2*(baseBuf[0]-128): 2*(128-baseBuf[0]);
  int8_t speedRatio = baseBuf[1] >= 128 ? 2*(baseBuf[1]-128): 2*(128-baseBuf[1]);
  
  // For forward
  if (speedDirection > 0) {
    switch (turnDirection) {
      case -1:
        // Left Turn
        analogWrite(LEFT_MOTOR_FORWARD, speedRatio);
        analogWrite(RIGHT_MOTOR_FORWARD, speedRatio * turnRatio);
        break;

      case 1:
        // Right Turn
        analogWrite(LEFT_MOTOR_FORWARD, speedRatio * turnRatio);
        analogWrite(RIGHT_MOTOR_FORWARD, speedRatio);
        break;

      default:
        // No turn
        analogWrite(LEFT_MOTOR_FORWARD, speedRatio);
        analogWrite(RIGHT_MOTOR_FORWARD, speedRatio);
        break;
    }
  } else {
    // For backwards
    switch (turnDirection) {
      case -1:
        // Left Turn
        analogWrite(LEFT_MOTOR_BACKWARD, speedRatio);
        analogWrite(RIGHT_MOTOR_BACKWARD, speedRatio * turnRatio);
        break;

      case 1:
        // Right Turn
        analogWrite(LEFT_MOTOR_BACKWARD, speedRatio * turnRatio);
        analogWrite(RIGHT_MOTOR_BACKWARD, speedRatio);
        break;

      default:
        // No turn
        analogWrite(LEFT_MOTOR_BACKWARD, speedRatio);
        analogWrite(RIGHT_MOTOR_BACKWARD, speedRatio);
        break;
    }
  }
}

