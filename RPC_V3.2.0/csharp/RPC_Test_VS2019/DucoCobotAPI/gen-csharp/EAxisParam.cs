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
public partial class EAxisParam : TBase
{

  public int Type { get; set; }

  public int Mode { get; set; }

  public double Ref_velo { get; set; }

  public double Ref_acc { get; set; }

  public double Ref_jerk { get; set; }

  public double Max_velo { get; set; }

  public double Max_acc { get; set; }

  public int Encoder_type { get; set; }

  public int Encoder_resolution { get; set; }

  public double Position_bias { get; set; }

  public double Screw_lead { get; set; }

  public bool Invert { get; set; }

  public List<double> Position_limit { get; set; }

  public EAxisParam() {
  }

  public EAxisParam(int type, int mode, double ref_velo, double ref_acc, double ref_jerk, double max_velo, double max_acc, int encoder_type, int encoder_resolution, double position_bias, double screw_lead, bool invert, List<double> position_limit) : this() {
    this.Type = type;
    this.Mode = mode;
    this.Ref_velo = ref_velo;
    this.Ref_acc = ref_acc;
    this.Ref_jerk = ref_jerk;
    this.Max_velo = max_velo;
    this.Max_acc = max_acc;
    this.Encoder_type = encoder_type;
    this.Encoder_resolution = encoder_resolution;
    this.Position_bias = position_bias;
    this.Screw_lead = screw_lead;
    this.Invert = invert;
    this.Position_limit = position_limit;
  }

  public void Read (TProtocol iprot)
  {
    iprot.IncrementRecursionDepth();
    try
    {
      bool isset_type = false;
      bool isset_mode = false;
      bool isset_ref_velo = false;
      bool isset_ref_acc = false;
      bool isset_ref_jerk = false;
      bool isset_max_velo = false;
      bool isset_max_acc = false;
      bool isset_encoder_type = false;
      bool isset_encoder_resolution = false;
      bool isset_position_bias = false;
      bool isset_screw_lead = false;
      bool isset_invert = false;
      bool isset_position_limit = false;
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
            if (field.Type == TType.I32) {
              Type = iprot.ReadI32();
              isset_type = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 2:
            if (field.Type == TType.I32) {
              Mode = iprot.ReadI32();
              isset_mode = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 3:
            if (field.Type == TType.Double) {
              Ref_velo = iprot.ReadDouble();
              isset_ref_velo = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 4:
            if (field.Type == TType.Double) {
              Ref_acc = iprot.ReadDouble();
              isset_ref_acc = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 5:
            if (field.Type == TType.Double) {
              Ref_jerk = iprot.ReadDouble();
              isset_ref_jerk = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 6:
            if (field.Type == TType.Double) {
              Max_velo = iprot.ReadDouble();
              isset_max_velo = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 7:
            if (field.Type == TType.Double) {
              Max_acc = iprot.ReadDouble();
              isset_max_acc = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 8:
            if (field.Type == TType.I32) {
              Encoder_type = iprot.ReadI32();
              isset_encoder_type = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 9:
            if (field.Type == TType.I32) {
              Encoder_resolution = iprot.ReadI32();
              isset_encoder_resolution = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 10:
            if (field.Type == TType.Double) {
              Position_bias = iprot.ReadDouble();
              isset_position_bias = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 11:
            if (field.Type == TType.Double) {
              Screw_lead = iprot.ReadDouble();
              isset_screw_lead = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 12:
            if (field.Type == TType.Bool) {
              Invert = iprot.ReadBool();
              isset_invert = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 13:
            if (field.Type == TType.List) {
              {
                Position_limit = new List<double>();
                TList _list132 = iprot.ReadListBegin();
                for( int _i133 = 0; _i133 < _list132.Count; ++_i133)
                {
                  double _elem134;
                  _elem134 = iprot.ReadDouble();
                  Position_limit.Add(_elem134);
                }
                iprot.ReadListEnd();
              }
              isset_position_limit = true;
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
      if (!isset_type)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Type not set");
      if (!isset_mode)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Mode not set");
      if (!isset_ref_velo)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Ref_velo not set");
      if (!isset_ref_acc)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Ref_acc not set");
      if (!isset_ref_jerk)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Ref_jerk not set");
      if (!isset_max_velo)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Max_velo not set");
      if (!isset_max_acc)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Max_acc not set");
      if (!isset_encoder_type)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Encoder_type not set");
      if (!isset_encoder_resolution)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Encoder_resolution not set");
      if (!isset_position_bias)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Position_bias not set");
      if (!isset_screw_lead)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Screw_lead not set");
      if (!isset_invert)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Invert not set");
      if (!isset_position_limit)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Position_limit not set");
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
      TStruct struc = new TStruct("EAxisParam");
      oprot.WriteStructBegin(struc);
      TField field = new TField();
      field.Name = "type";
      field.Type = TType.I32;
      field.ID = 1;
      oprot.WriteFieldBegin(field);
      oprot.WriteI32(Type);
      oprot.WriteFieldEnd();
      field.Name = "mode";
      field.Type = TType.I32;
      field.ID = 2;
      oprot.WriteFieldBegin(field);
      oprot.WriteI32(Mode);
      oprot.WriteFieldEnd();
      field.Name = "ref_velo";
      field.Type = TType.Double;
      field.ID = 3;
      oprot.WriteFieldBegin(field);
      oprot.WriteDouble(Ref_velo);
      oprot.WriteFieldEnd();
      field.Name = "ref_acc";
      field.Type = TType.Double;
      field.ID = 4;
      oprot.WriteFieldBegin(field);
      oprot.WriteDouble(Ref_acc);
      oprot.WriteFieldEnd();
      field.Name = "ref_jerk";
      field.Type = TType.Double;
      field.ID = 5;
      oprot.WriteFieldBegin(field);
      oprot.WriteDouble(Ref_jerk);
      oprot.WriteFieldEnd();
      field.Name = "max_velo";
      field.Type = TType.Double;
      field.ID = 6;
      oprot.WriteFieldBegin(field);
      oprot.WriteDouble(Max_velo);
      oprot.WriteFieldEnd();
      field.Name = "max_acc";
      field.Type = TType.Double;
      field.ID = 7;
      oprot.WriteFieldBegin(field);
      oprot.WriteDouble(Max_acc);
      oprot.WriteFieldEnd();
      field.Name = "encoder_type";
      field.Type = TType.I32;
      field.ID = 8;
      oprot.WriteFieldBegin(field);
      oprot.WriteI32(Encoder_type);
      oprot.WriteFieldEnd();
      field.Name = "encoder_resolution";
      field.Type = TType.I32;
      field.ID = 9;
      oprot.WriteFieldBegin(field);
      oprot.WriteI32(Encoder_resolution);
      oprot.WriteFieldEnd();
      field.Name = "position_bias";
      field.Type = TType.Double;
      field.ID = 10;
      oprot.WriteFieldBegin(field);
      oprot.WriteDouble(Position_bias);
      oprot.WriteFieldEnd();
      field.Name = "screw_lead";
      field.Type = TType.Double;
      field.ID = 11;
      oprot.WriteFieldBegin(field);
      oprot.WriteDouble(Screw_lead);
      oprot.WriteFieldEnd();
      field.Name = "invert";
      field.Type = TType.Bool;
      field.ID = 12;
      oprot.WriteFieldBegin(field);
      oprot.WriteBool(Invert);
      oprot.WriteFieldEnd();
      if (Position_limit == null)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Position_limit not set");
      field.Name = "position_limit";
      field.Type = TType.List;
      field.ID = 13;
      oprot.WriteFieldBegin(field);
      {
        oprot.WriteListBegin(new TList(TType.Double, Position_limit.Count));
        foreach (double _iter135 in Position_limit)
        {
          oprot.WriteDouble(_iter135);
        }
        oprot.WriteListEnd();
      }
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
    StringBuilder __sb = new StringBuilder("EAxisParam(");
    __sb.Append(", Type: ");
    __sb.Append(Type);
    __sb.Append(", Mode: ");
    __sb.Append(Mode);
    __sb.Append(", Ref_velo: ");
    __sb.Append(Ref_velo);
    __sb.Append(", Ref_acc: ");
    __sb.Append(Ref_acc);
    __sb.Append(", Ref_jerk: ");
    __sb.Append(Ref_jerk);
    __sb.Append(", Max_velo: ");
    __sb.Append(Max_velo);
    __sb.Append(", Max_acc: ");
    __sb.Append(Max_acc);
    __sb.Append(", Encoder_type: ");
    __sb.Append(Encoder_type);
    __sb.Append(", Encoder_resolution: ");
    __sb.Append(Encoder_resolution);
    __sb.Append(", Position_bias: ");
    __sb.Append(Position_bias);
    __sb.Append(", Screw_lead: ");
    __sb.Append(Screw_lead);
    __sb.Append(", Invert: ");
    __sb.Append(Invert);
    __sb.Append(", Position_limit: ");
    __sb.Append(Position_limit);
    __sb.Append(")");
    return __sb.ToString();
  }

}
