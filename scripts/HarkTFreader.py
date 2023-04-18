#!/usr/bin/python3
# -*- coding: utf-8 -*-
import numpy
import zipfile
import xml.etree.ElementTree as etree

class HarkTFreader(object):
    def __init__(self, filename=""):
        self.valid = False
        self.source_positions = []
        if filename:
            with zipfile.ZipFile(filename, 'r') as TF:
                for info in TF.infolist():
                    if "transferFunction/source.xml" in info.filename:
                        source_xml = TF.read("transferFunction/source.xml")
                        tree = etree.fromstring(source_xml)
                        for pos in tree.find("positions").findall("position"):
                            self.source_positions.append(pos.attrib)
                        if self.source_positions:
                            self.valid = True

    def cart2polar(self, cart):
        return 180.0 / numpy.pi * numpy.arctan2(cart[1], cart[0])

    def source_search_by_ids(self, ids=[], debug=False):
        """
        This method returns Cartesian coordinates.
        If Polar coordiantes are required, call the cart2polar method from externally.
        """
        if isinstance(ids, int) or isinstance(ids, float):
            ids = list(int(round(ids, 0)))
        filtered_src = []
        for src in self.source_positions:
            if int(src["id"]) in ids:
                if debug:
                    print("id {} found".format(src["id"]))
                filtered_src.append(src)
        if debug:
            print("filter ids : ", ids)
        return filtered_src

    def source_search_by_angles(self, range_min=-180.0, range_max=180.0, debug=False):
        """
        For the moment only azimuth is supported.
        """
        range_min = float(range_min)
        range_max = float(range_max)
        single = True if range_min == range_max else False
        while range_max >= 180.0:
            range_max -= 360.0
        while range_max < -180.0:
            range_max += 360.0
        while range_min >= 180.0:
            range_min -= 360.0
        while range_min < -180.0:
            range_min += 360.0
        boundary = True if range_min > range_max else False
        eps = 0.01
        if range_min == range_max:
            if single:
                # for filter range normalize to [direction-eps,direction+eps]
                # Notes:
                #   The loose range setting eliminates the influence of the coordinate accuracy
                #   of the source.xml file in the TransferFunction zip file.
                range_min -= eps
                range_max += eps
            else:
                # for filter range normalize to [-180,180) --- This is no filter.
                # Notes:
                #   When setting the range to the overall orientation,
                #   it should be correctly set to "min=-180,max=179 or min=0,max=359 or etc...",
                #   but humans set it as "min=-180,max=180 or min=0,max=360 or etc..." often from
                #   sensory point of view.
                range_min = -180.0
                range_max = 180.0 - eps
        filtered_src = []
        for src in self.source_positions:
            cart = (float(src["y"]), float(src["x"]))
            azimuth = self.cart2polar(cart)
            while azimuth >= 180.0:
                azimuth -= 360.0
            while azimuth < -180.0:
                azimuth += 360.0
            if (not boundary) and (range_min <= azimuth <= range_max):
                if debug:
                    print("azimuth {} is within range".format(round(azimuth, 1)))
                filtered_src.append(src)
            elif (boundary) and ((range_min <= azimuth <= range_max+360.0) or (range_min-360.0 <= azimuth <= range_max)):
                if debug:
                    print("azimuth {} is within range (including -180/180 boundary)".format(round(azimuth, 1)))
                filtered_src.append(src)
            else:
                if debug:
                    print("azimuth {} is out of range".format(round(azimuth, 1)))
        if debug:
            print("filter range : ({} - {})".format(round(range_min, 1), round(range_max, 1)))
        return filtered_src

if __name__ == '__main__':
    tf_reader = HarkTFreader("hark_conf/tamago_rectf.zip")
    #tf_reader = HarkTFreader("hark_conf/tamago_geotf.zip")
    if tf_reader.valid:
        #print(tf_reader.source_positions)
        #print([int(x["id"]) for x in tf_reader.source_search_by_angles(0, 360)])
        #print([int(x["id"]) for x in tf_reader.source_search_by_angles(0, 359)])
        #print([int(x["id"]) for x in tf_reader.source_search_by_angles(-180, 180)])
        #print([int(x["id"]) for x in tf_reader.source_search_by_angles(180, 180)])
        #print([int(x["id"]) for x in tf_reader.source_search_by_angles(-90, 90)])
        #print([int(x["id"]) for x in tf_reader.source_search_by_angles(90, -90)])
        #print([int(x["id"]) for x in tf_reader.source_search_by_angles(30, 30)])
        #print([int(x["id"]) for x in tf_reader.source_search_by_angles(-30, 30, True)])
        #print([round(tf_reader.cart2polar(cart=(float(x["y"]), float(x["x"]))), 1) for x in tf_reader.source_search_by_ids([2, 4, 6])])
        #print([round(tf_reader.cart2polar(cart=(float(x["y"]), float(x["x"]))), 1) for x in   tf_reader.source_search_by_ids([2, 4, 6, 90])])
        pass

