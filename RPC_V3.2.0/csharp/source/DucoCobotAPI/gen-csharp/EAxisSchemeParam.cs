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
public partial class EAxisSchemeParam : TBase
{

  public int Type { get; set; }

  public List<int> Axis_link { get; set; }

  public List<double> Base { get; set; }

  public List<List<double>> Dh { get; set; }

  public int Install { get; set; }

  public EAxisSchemeParam() {
  }

  public EAxisSchemeParam(int type, List<int> axis_link, List<double> @base, List<List<double>> dh, int install) : this() {
    this.Type = type;
    this.Axis_link = axis_link;
    this.Base = @base;
    this.Dh = dh;
    this.Install = install;
  }

  public void Read (TProtocol iprot)
  {
    iprot.IncrementRecursionDepth();
    try
    {
      bool isset_type = false;
      bool isset_axis_link = false;
      bool isset_base = false;
      bool isset_dh = false;
      bool isset_install = false;
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
            if (field.Type == TType.List) {
              {
                Axis_link = new List<int>();
                TList _list136 = iprot.ReadListBegin();
                for( int _i137 = 0; _i137 < _list136.Count; ++_i137)
                {
                  int _elem138;
                  _elem138 = iprot.ReadI32();
                  Axis_link.Add(_elem138);
                }
                iprot.ReadListEnd();
              }
              isset_axis_link = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 3:
            if (field.Type == TType.List) {
              {
                Base = new List<double>();
                TList _list139 = iprot.ReadListBegin();
                for( int _i140 = 0; _i140 < _list139.Count; ++_i140)
                {
                  double _elem141;
                  _elem141 = iprot.ReadDouble();
                  Base.Add(_elem141);
                }
                iprot.ReadListEnd();
              }
              isset_base = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 4:
            if (field.Type == TType.List) {
              {
                Dh = new List<List<double>>();
                TList _list142 = iprot.ReadListBegin();
                for( int _i143 = 0; _i143 < _list142.Count; ++_i143)
                {
                  List<double> _elem144;
                  {
                    _elem144 = new List<double>();
                    TList _list145 = iprot.ReadListBegin();
                    for( int _i146 = 0; _i146 < _list145.Count; ++_i146)
                    {
                      double _elem147;
                      _elem147 = iprot.ReadDouble();
                      _elem144.Add(_elem147);
                    }
                    iprot.ReadListEnd();
                  }
                  Dh.Add(_elem144);
                }
                iprot.ReadListEnd();
              }
              isset_dh = true;
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 5:
            if (field.Type == TType.I32) {
              Install = iprot.ReadI32();
              isset_install = true;
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
      if (!isset_axis_link)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Axis_link not set");
      if (!isset_base)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Base not set");
      if (!isset_dh)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Dh not set");
      if (!isset_install)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Install not set");
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
      TStruct struc = new TStruct("EAxisSchemeParam");
      oprot.WriteStructBegin(struc);
      TField field = new TField();
      field.Name = "type";
      field.Type = TType.I32;
      field.ID = 1;
      oprot.WriteFieldBegin(field);
      oprot.WriteI32(Type);
      oprot.WriteFieldEnd();
      if (Axis_link == null)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Axis_link not set");
      field.Name = "axis_link";
      field.Type = TType.List;
      field.ID = 2;
      oprot.WriteFieldBegin(field);
      {
        oprot.WriteListBegin(new TList(TType.I32, Axis_link.Count));
        foreach (int _iter148 in Axis_link)
        {
          oprot.WriteI32(_iter148);
        }
        oprot.WriteListEnd();
      }
      oprot.WriteFieldEnd();
      if (Base == null)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Base not set");
      field.Name = "base";
      field.Type = TType.List;
      field.ID = 3;
      oprot.WriteFieldBegin(field);
      {
        oprot.WriteListBegin(new TList(TType.Double, Base.Count));
        foreach (double _iter149 in Base)
        {
          oprot.WriteDouble(_iter149);
        }
        oprot.WriteListEnd();
      }
      oprot.WriteFieldEnd();
      if (Dh == null)
        throw new TProtocolException(TProtocolException.INVALID_DATA, "required field Dh not set");
      field.Name = "dh";
      field.Type = TType.List;
      field.ID = 4;
      oprot.WriteFieldBegin(field);
      {
        oprot.WriteListBegin(new TList(TType.List, Dh.Count));
        foreach (List<double> _iter150 in Dh)
        {
          {
            oprot.WriteListBegin(new TList(TType.Double, _iter150.Count));
            foreach (double _iter151 in _iter150)
            {
              oprot.WriteDouble(_iter151);
            }
            oprot.WriteListEnd();
          }
        }
        oprot.WriteListEnd();
      }
      oprot.WriteFieldEnd();
      field.Name = "install";
      field.Type = TType.I32;
      field.ID = 5;
      oprot.WriteFieldBegin(field);
      oprot.WriteI32(Install);
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
    StringBuilder __sb = new StringBuilder("EAxisSchemeParam(");
    __sb.Append(", Type: ");
    __sb.Append(Type);
    __sb.Append(", Axis_link: ");
    __sb.Append(Axis_link);
    __sb.Append(", Base: ");
    __sb.Append(Base);
    __sb.Append(", Dh: ");
    __sb.Append(Dh);
    __sb.Append(", Install: ");
    __sb.Append(Install);
    __sb.Append(")");
    return __sb.ToString();
  }

}

