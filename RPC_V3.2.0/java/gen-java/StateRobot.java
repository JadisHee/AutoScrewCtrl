/**
 * Autogenerated by Thrift Compiler (0.13.0)
 *
 * DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
 *  @generated
 */

@javax.annotation.Generated(value = "Autogenerated by Thrift Compiler (0.13.0)", date = "2024-01-19")
public enum StateRobot implements org.apache.thrift.TEnum {
  SR_Start(0),
  SR_Initialize(1),
  SR_Logout(2),
  SR_Login(3),
  SR_PowerOff(4),
  SR_Disable(5),
  SR_Enable(6);

  private final int value;

  private StateRobot(int value) {
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
  public static StateRobot findByValue(int value) { 
    switch (value) {
      case 0:
        return SR_Start;
      case 1:
        return SR_Initialize;
      case 2:
        return SR_Logout;
      case 3:
        return SR_Login;
      case 4:
        return SR_PowerOff;
      case 5:
        return SR_Disable;
      case 6:
        return SR_Enable;
      default:
        return null;
    }
  }
}