/**
 * Autogenerated by Thrift Compiler (0.13.0)
 *
 * DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
 *  @generated
 */
using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.IO;
using Thrift;
using Thrift.Collections;
using System.Runtime.Serialization;
using Thrift.Protocol;
using Thrift.Transport;


#if !SILVERLIGHT
[Serializable]
#endif
public partial class RealTimeControlData : TBase
{

  public List<double> Joint_pos_cmd { get; set; }

  public List<double> Joint_vel_cmd { get; set; }

  public List<double> Joint_torq_cmd { get; set; }

  public List<double> Cart_pos_tool_wobj_cmd { get; set; }

  public List<double> Cart_vel_tool_wobj_cmd { get; set; }

  public List<double> Cart_ft_cmd { get; set; }

  public bool Status { get; set; }

  public RealTimeControlData() {
  }

  public RealTimeControlData(List<double> joint_pos_cmd, List<double> joint_vel_cmd, List<double> joint_torq_cmd, List<double> cart_pos_tool_wobj_cmd, List<double> cart_vel_tool_wobj_cmd, List<double> cart_ft_cmd, bool status) : this() {
    this.Joint_pos_cmd = joint_pos_cmd;
    this.Joint_vel_cmd = joint_vel_cmd;
    this.Joint_torq_cmd = joint_torq_cmd;
    this.Cart_pos_tool_wobj_cmd = cart_pos_tool_wobj_cmd;
    this.Cart_vel_tool_wobj_cmd = cart_vel_tool_wobj_cmd;
    this.Cart_ft_cmd = cart_ft_cmd;
    this.Status = status;
  }

  public void Read (TProtocol iprot)
  {
    iprot.IncrementRecursionDepth();
    try
    {
      bool isset_joint_pos_cmd = false;
      bool isset_joint_vel_cmd = false;
      bool isset_joint_torq_cmd = false;
      bool isset_cart_pos_tool_wobj_cmd = false;
      bool isset_cart_vel_tool_wobj_cmd = false;
      bool isset_cart_ft_cmd = false;
      bool isset_status = false;
      TField field;
      iprot.ReadStructBegin();
      while (true)
      {
        field = iprot.ReadFieldBegin();
        if (field.Type == TType.Stop) { 
          break;
        }
        switch (field.ID)
        {
          case 1:
            if (field.Type == TType.List) {
              {
                Joint_pos_cmd = new List<double>();
                TList _list160 = iprot.ReadListBegin();
                for( int _i161 = 0; _i161 < _list160.Count; ++_i161)
                {
                  double _elem162;
                  _elem162 = iprot.ReadDouble();
                  Joint_pos_cmd.Add(_elem162);
                }
                iprot.ReadListEnd();
              }
              isset_joint_pos_cmd = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 2:
            if (field.Type == TType.List) {
              {
                Joint_vel_cmd = new List<double>();
                TList _list163 = iprot.ReadListBegin();
                for( int _i164 = 0; _i164 < _list163.Count; ++_i164)
                {
                  double _elem165;
                  _elem165 = iprot.ReadDouble();
                  Joint_vel_cmd.Add(_elem165);
                }
                iprot.ReadListEnd();
              }
              isset_joint_vel_cmd = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 3:
            if (field.Type == TType.List) {
              {
                Joint_torq_cmd = new List<double>();
                TList _list166 = iprot.ReadListBegin();
                for( int _i167 = 0; _i167 < _list166.Count; ++_i167)
                {
                  double _elem168;
                  _elem168 = iprot.ReadDouble();
                  Joint_torq_cmd.Add(_elem168);
                }
                iprot.ReadListEnd();
              }
              isset_joint_torq_cmd = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 4:
            if (field.Type == TType.List) {
              {
                Cart_pos_tool_wobj_cmd = new List<double>();
                TList _list169 = iprot.ReadListBegin();
                for( int _i170 = 0; _i170 < _list169.Count; ++_i170)
                {
                  double _elem171;
                  _elem171 = iprot.ReadDouble();
                  Cart_pos_tool_wobj_cmd.Add(_elem171);
                }
                iprot.ReadListEnd();
              }
              isset_cart_pos_tool_wobj_cmd = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 5:
            if (field.Type == TType.List) {
              {
                Cart_vel_tool_wobj_cmd = new List<double>();
                TList _list172 = iprot.ReadListBegin();
                for( int _i173 = 0; _i173 < _list172.Count; ++_i173)
                {
                  double _elem174;
                  _elem174 = iprot.ReadDouble();
                  Cart_vel_tool_wobj_cmd.Add(_elem174);
                }
                iprot.ReadListEnd();
              }
              isset_cart_vel_tool_wobj_cmd = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 6:
            if (field.Type == TType.List) {
              {
                Cart_ft_cmd = new List<double>();
                TList _list175 = iprot.ReadListBegin();
                for( int _i176 = 0; _i176 < _list175.Count; ++_i176)
                {
                  double _elem177;
                  _elem177 = iprot.ReadDouble();
                  Cart_ft_cmd.Add(_elem177);
                }
                iprot.ReadListEnd();
              }
              isset_cart_ft_cmd = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 7:
            if (field.Type == TType.Bool) {
              Status = iprot.ReadBool();
              isset_status = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          default: 
            TProtocolUtil.Skip(iprot, field.Type);
            break;
        }
        iprot.ReadFieldEnd();
      }
      iprot.ReadStructEnd();
      if (!isset_joint_pos_cmd)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Joint_pos_cmd not set");
      if (!isset_joint_vel_cmd)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Joint_vel_cmd not set");
      if (!isset_joint_torq_cmd)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Joint_torq_cmd not set");
      if (!isset_cart_pos_tool_wobj_cmd)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Cart_pos_tool_wobj_cmd not set");
      if (!isset_cart_vel_tool_wobj_cmd)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Cart_vel_tool_wobj_cmd not set");
      if (!isset_cart_ft_cmd)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Cart_ft_cmd not set");
      if (!isset_status)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Status not set");
    }
    finally
    {
      iprot.DecrementRecursionDepth();
    }
  }

  public void Write(TProtocol oprot) {
    oprot.IncrementRecursionDepth();
    try
    {
      TStruct struc = new TStruct("RealTimeControlData");
      oprot.WriteStructBegin(struc);
      TField field = new TField();
      if (Joint_pos_cmd == null)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Joint_pos_cmd not set");
      field.Name = "joint_pos_cmd";
      field.Type = TType.List;
      field.ID = 1;
      oprot.WriteFieldBegin(field);
      {
        oprot.WriteListBegin(new TList(TType.Double, Joint_pos_cmd.Count));
        foreach (double _iter178 in Joint_pos_cmd)
        {
          oprot.WriteDouble(_iter178);
        }
        oprot.WriteListEnd();
      }
      oprot.WriteFieldEnd();
      if (Joint_vel_cmd == null)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Joint_vel_cmd not set");
      field.Name = "joint_vel_cmd";
      field.Type = TType.List;
      field.ID = 2;
      oprot.WriteFieldBegin(field);
      {
        oprot.WriteListBegin(new TList(TType.Double, Joint_vel_cmd.Count));
        foreach (double _iter179 in Joint_vel_cmd)
        {
          oprot.WriteDouble(_iter179);
        }
        oprot.WriteListEnd();
      }
      oprot.WriteFieldEnd();
      if (Joint_torq_cmd == null)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Joint_torq_cmd not set");
      field.Name = "joint_torq_cmd";
      field.Type = TType.List;
      field.ID = 3;
      oprot.WriteFieldBegin(field);
      {
        oprot.WriteListBegin(new TList(TType.Double, Joint_torq_cmd.Count));
        foreach (double _iter180 in Joint_torq_cmd)
        {
          oprot.WriteDouble(_iter180);
        }
        oprot.WriteListEnd();
      }
      oprot.WriteFieldEnd();
      if (Cart_pos_tool_wobj_cmd == null)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Cart_pos_tool_wobj_cmd not set");
      field.Name = "cart_pos_tool_wobj_cmd";
      field.Type = TType.List;
      field.ID = 4;
      oprot.WriteFieldBegin(field);
      {
        oprot.WriteListBegin(new TList(TType.Double, Cart_pos_tool_wobj_cmd.Count));
        foreach (double _iter181 in Cart_pos_tool_wobj_cmd)
        {
          oprot.WriteDouble(_iter181);
        }
        oprot.WriteListEnd();
      }
      oprot.WriteFieldEnd();
      if (Cart_vel_tool_wobj_cmd == null)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Cart_vel_tool_wobj_cmd not set");
      field.Name = "cart_vel_tool_wobj_cmd";
      field.Type = TType.List;
      field.ID = 5;
      oprot.WriteFieldBegin(field);
      {
        oprot.WriteListBegin(new TList(TType.Double, Cart_vel_tool_wobj_cmd.Count));
        foreach (double _iter182 in Cart_vel_tool_wobj_cmd)
        {
          oprot.WriteDouble(_iter182);
        }
        oprot.WriteListEnd();
      }
      oprot.WriteFieldEnd();
      if (Cart_ft_cmd == null)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Cart_ft_cmd not set");
      field.Name = "cart_ft_cmd";
      field.Type = TType.List;
      field.ID = 6;
      oprot.WriteFieldBegin(field);
      {
        oprot.WriteListBegin(new TList(TType.Double, Cart_ft_cmd.Count));
        foreach (double _iter183 in Cart_ft_cmd)
        {
          oprot.WriteDouble(_iter183);
        }
        oprot.WriteListEnd();
      }
      oprot.WriteFieldEnd();
      field.Name = "status";
      field.Type = TType.Bool;
      field.ID = 7;
      oprot.WriteFieldBegin(field);
      oprot.WriteBool(Status);
      oprot.WriteFieldEnd();
      oprot.WriteFieldStop();
      oprot.WriteStructEnd();
    }
    finally
    {
      oprot.DecrementRecursionDepth();
    }
  }

  public override string ToString() {
    StringBuilder __sb = new StringBuilder("RealTimeControlData(");
    __sb.Append(", Joint_pos_cmd: ");
    __sb.Append(Joint_pos_cmd);
    __sb.Append(", Joint_vel_cmd: ");
    __sb.Append(Joint_vel_cmd);
    __sb.Append(", Joint_torq_cmd: ");
    __sb.Append(Joint_torq_cmd);
    __sb.Append(", Cart_pos_tool_wobj_cmd: ");
    __sb.Append(Cart_pos_tool_wobj_cmd);
    __sb.Append(", Cart_vel_tool_wobj_cmd: ");
    __sb.Append(Cart_vel_tool_wobj_cmd);
    __sb.Append(", Cart_ft_cmd: ");
    __sb.Append(Cart_ft_cmd);
    __sb.Append(", Status: ");
    __sb.Append(Status);
    __sb.Append(")");
    return __sb.ToString();
  }

}
