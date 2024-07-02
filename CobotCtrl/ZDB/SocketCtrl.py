import xml.etree.ElementTree as ET

class CommunicateData:
    
    # TypeData = 2

    Command_Data = 1
    Angle_Data = 0
    Error_Data = ''


    # def RecvData(self):
    #     Message = ET.Element("Message")
    #     Type = ET.SubElement(Message,"Type")
    #     Command = ET.SubElement(Message,"Type")

    #     Type.text = str(self.TypeData)
    #     Command.text = str(self.Command_Data)



    def XmlData(self):
        Message = ET.Element("Message")
        
        Command = ET.SubElement(Message,"Command")
        Angle = ET.SubElement(Message,"Angle")
        # ProductType = ET.SubElement(Message,"Type")
        ErrorMsg = ET.SubElement(Message,"ErrorMsg")

        # Type.text = str(self.TypeData)
        Command.text = str(self.Command_Data)
        Angle.text = str(self.Angle_Data)
        ErrorMsg.text = self.Error_Data

        xml = ET.tostring(Message,encoding="unicode")
        return xml