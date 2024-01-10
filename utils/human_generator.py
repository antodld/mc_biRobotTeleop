import xml.etree.ElementTree as ET
from xml.dom.minidom import parse, parseString
from enum import Enum
import yaml

class link_type(Enum):
    CYLINDER = 0
    BOX = 1
    SPHERE = 2
class link_param:
    def __init__(self,name,childs,shape) -> None:
        self.name :str = name #link name
        self.childs  = childs #links child ignoring virtual joints
        if(len(shape) == 2):
            self.type = link_type.CYLINDER
            self.l = shape[0]
            self.r = shape[1]
        elif(len(shape) == 1):
            self.type = link_type.SPHERE
            self.r = shape[0]
        else:
            self.type = link_type.BOX
            self.x, self.y, self.z = shape[0], shape[1], shape[2]


def update_link(param : link_param):
    global root
    if param.type == link_type.BOX:
        update_box_link(param)
        return
    if param.type == link_type.SPHERE:
        update_sphere_link(param)
        return

    for link in root.iter("link"):
        if link.get("name") == param.name:
            for cyl in link.iter("cylinder"):
                cyl.set("length",str(param.l))
                cyl.set("radius",str(param.r))
            for ori in link.iter("origin"):
                ori.set("xyz","0. 0. " + str(-param.l/2))
            
    if len(param.childs) != 0:
        for joint in root.iter("joint"):
            for child in joint.iter("child"):
                if child.get("link") == param.childs[0]:
                    for origin in joint.iter("origin"):
                        origin.set("xyz","0. 0. " + str(-param.l))

def update_box_link(param: link_param):
    for link in root.iter("link"):
        if link.get("name") == param.name:
            for box in link.iter("box"):
                box.set("size",str(param.x) + " " + str(param.y) + " " + str(param.z))
            for ori in link.iter("origin"):
                ori.set("xyz","0. 0. " + str(param.z/2))
    
    for link_child in param.childs:
        for joint in root.iter("joint"):
            for child in joint.iter("child"):
                if child.get("link") == link_child:
                    
                    for origin in joint.iter("origin"):
                        if link_child[0] == "R":
                            cyl_param = links["R_ARM_LINK"]
                            origin.set("xyz","0. " + str(-( (param.y/2) + cyl_param.r )) + " " + str(param.z))
                        elif link_child[0] == "L":
                            cyl_param = links["L_ARM_LINK"]
                            origin.set("xyz","0. " +  str(( (param.y/2) + cyl_param.r )) + " " + str(param.z))
                        else:
                            sphere_param = links["HEAD_LINK"]
                            origin.set("xyz","0. 0. " + str(param.z + sphere_param.r))

def update_sphere_link(param: link_param):
    for link in root.iter("link"):
        if link.get("name") == param.name:
            for box in link.iter("sphere"):
                box.set("radius",str(param.r))
    
def load():
    with open("config.yaml") as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        tree = data["tree"]
        links_param = {}
        for link in data["links"]:
            name = link["name"]
            links_param[name] = link_param(name,link["child"],link["dimension"])
        
        return links_param, tree

path = "/home/antonin/devel/src/catkin_data_ws/src/simple_human_description/urdf/simple_human.urdf"

arbre = ET.parse(path)

root = arbre.getroot()

links, trees = load()

indx = 0
for tree in trees:
    for link_name in tree[indx:]:
        update_link(links[link_name])
    indx = 1

arbre.write("/home/antonin/devel/src/catkin_data_ws/src/simple_human_description/urdf/simple_human.urdf")
