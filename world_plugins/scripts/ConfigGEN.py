# -*- coding: UTF-8 -*-
from lxml import etree as ET
import os


# 生成config文件
def generate_config(
    model_name, save_path='', description_text='My Generate Model for Gazebo'
):
    '''生成model的config文件并保存

    params:
        model_name: model名称
        save_path: 文件保存路径
        description_text: model描述
    '''

    model = ET.Element("model")

    name = ET.SubElement(model, "name")
    name.text = model_name

    version = ET.SubElement(model, "version")
    version.text = "1.0"

    sdf = ET.SubElement(model, "sdf", version="1.6")
    sdf.text = "model.sdf"

    author = ET.SubElement(model, "author")

    name = ET.SubElement(author, "name")
    name.text = "Wenhao"

    email = ET.SubElement(author, "email")
    email.text = "fengqlwy@gmail.com"

    description = ET.SubElement(model, "description")
    description.text = description_text

    # print ET.tostring(model,pretty_print=True,xml_declaration=True)

    tree = ET.ElementTree(model)
    tree.write(
        os.path.join(save_path, 'model.config'),
        pretty_print=True,
        xml_declaration=True,
    )
