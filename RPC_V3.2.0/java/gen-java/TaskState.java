/**
 * Autogenerated by Thrift Compiler (0.13.0)
 *
 * DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
 *  @generated
 */

@javax.annotation.Generated(value = "Autogenerated by Thrift Compiler (0.13.0)", date = "2024-01-19")
public enum TaskState implements org.apache.thrift.TEnum {
  ST_Idle(0),
  ST_Running(1),
  ST_Paused(2),
  ST_Stopped(3),
  ST_Finished(4),
  ST_Interrupt(5),
  ST_Error(6),
  ST_Illegal(7),
  ST_ParameterMismatch(8);

  private final int value;

  private TaskState(int value) {
    this.value = value;
  }

  /**
   * Get the integer value of this enum value, as defined in the Thrift IDL.
   */
  public int getValue() {
    return value;
  }

  /**
   * Find a the enum type by its integer value, as defined in the Thrift IDL.
   * @return null if the value is not found.
   */
  @org.apache.thrift.annotation.Nullable
  public static TaskState findByValue(int value) { 
    switch (value) {
      case 0:
        return ST_Idle;
      case 1:
        return ST_Running;
      case 2:
        return ST_Paused;
      case 3:
        return ST_Stopped;
      case 4:
        return ST_Finished;
      case 5:
        return ST_Interrupt;
      case 6:
        return ST_Error;
      case 7:
        return ST_Illegal;
      case 8:
        return ST_ParameterMismatch;
      default:
        return null;
    }
  }
}
