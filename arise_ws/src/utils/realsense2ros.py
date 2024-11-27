#!/usr/bin/env python
import numpy as np
import pyrealsense2 as rs
from rich import print


def ros2array(data,height,width,imgFormat):
    if imgFormat==np.uint8:
        return np.frombuffer(data, dtype=imgFormat).reshape(height, width, -1)
    elif imgFormat==np.uint16:
        return np.frombuffer(data, dtype=imgFormat).reshape(height, width)

def array2ros(img_bgr):
    return img_bgr.reshape(-1).tolist() 
    
    
def adjust_2D_point_to_boundary(point,width, height):

    point[0] = max(0, min(point[0], width - 1))
    point[1] = max(0, min(point[1], height - 1))

    return point

def is_pixel_in_line(curr, start, end):
    return ((end[0] >= start[0] and end[0] >= curr[0] and curr[0] >= start[0]) or
            (end[0] <= start[0] and end[0] <= curr[0] and curr[0] <= start[0])) and \
           ((end[1] >= start[1] and end[1] >= curr[1] and curr[1] >= start[1]) or
            (end[1] <= start[1] and end[1] <= curr[1] and curr[1] <= start[1]))

def next_pixel_in_line(curr, start, end):
    line_slope = (end[1] - start[1]) / (end[0] - start[0])
    if abs(end[0] - curr[0]) > abs(end[1] - curr[1]):
        curr[0] = end[0] + 1 if end[0] > curr[0] else end[0] - 1
        curr[1] = end[1] - line_slope * (end[0] - curr[0])
    else:
        curr[1] = end[1] + 1 if end[1] > curr[1] else end[1] - 1
        curr[0] = end[0] - ((end[1] + curr[1]) / line_slope)

def rs2_deproject_pixel_to_point(intrin, pixel, depth):
    point=np.zeros(3)
    x = (pixel[0] - intrin.ppx) / intrin.fx
    y = (pixel[1] - intrin.ppy) / intrin.fy

    xo = x
    yo = y

    if intrin.model == 2:  # RS2_DISTORTION_INVERSE_BROWN_CONRADY
        for _ in range(10):
            r2 = x * x + y * y
            icdist = 1 / (1 + ((intrin.coeffs[4] * r2 + intrin.coeffs[1]) * r2 + intrin.coeffs[0]) * r2)
            xq = x / icdist
            yq = y / icdist
            delta_x = 2 * intrin.coeffs[2] * xq * yq + intrin.coeffs[3] * (r2 + 2 * xq * xq)
            delta_y = 2 * intrin.coeffs[3] * xq * yq + intrin.coeffs[2] * (r2 + 2 * yq * yq)
            x = (xo - delta_x) * icdist
            y = (yo - delta_y) * icdist

    if intrin.model == 4:  # RS2_DISTORTION_BROWN_CONRADY
        for _ in range(10):
            r2 = x * x + y * y
            icdist = 1 / (1 + ((intrin.coeffs[4] * r2 + intrin.coeffs[1]) * r2 + intrin.coeffs[0]) * r2)
            delta_x = 2 * intrin.coeffs[2] * x * y + intrin.coeffs[3] * (r2 + 2 * x * x)
            delta_y = 2 * intrin.coeffs[3] * x * y + intrin.coeffs[2] * (r2 + 2 * y * y)
            x = (xo - delta_x) * icdist
            y = (yo - delta_y) * icdist

    if intrin.model == 5:  # RS2_DISTORTION_KANNALA_BRANDT4
        rd = np.sqrt(x * x + y * y)
        if rd < np.finfo(float).eps:
            rd = np.finfo(float).eps

        theta = rd
        theta2 = rd * rd
        for _ in range(4):
            f = theta * (1 + theta2 * (intrin.coeffs[0] + theta2 * (intrin.coeffs[1] + theta2 * (intrin.coeffs[2] + theta2 * intrin.coeffs[3])))) - rd
            if abs(f) < np.finfo(float).eps:
                break
            df = 1 + theta2 * (3 * intrin.coeffs[0] + theta2 * (5 * intrin.coeffs[1] + theta2 * (7 * intrin.coeffs[2] + 9 * theta2 * intrin.coeffs[3])))
            theta -= f / df
            theta2 = theta * theta

        r = np.tan(theta)
        x *= r / rd
        y *= r / rd

    if intrin.model == 3:  # RS2_DISTORTION_FTHETA
        rd = np.sqrt(x * x + y * y)
        if rd < np.finfo(float).eps:
            rd = np.finfo(float).eps
        r = np.tan(intrin.coeffs[0] * rd) / np.arctan(2 * np.tan(intrin.coeffs[0] / 2.0))
        x *= r / rd
        y *= r / rd

    point[0] = depth * x
    point[1] = depth * y
    point[2] = depth

    return point

def rs2_transform_point_to_point(extrin, from_point):
    to_point=np.zeros(3)

    to_point[0] = extrin.rotation[0] * from_point[0] + extrin.rotation[3] * from_point[1] + extrin.rotation[6] * from_point[2] + extrin.translation[0]
    to_point[1] = extrin.rotation[1] * from_point[0] + extrin.rotation[4] * from_point[1] + extrin.rotation[7] * from_point[2] + extrin.translation[1]
    to_point[2] = extrin.rotation[2] * from_point[0] + extrin.rotation[5] * from_point[1] + extrin.rotation[8] * from_point[2] + extrin.translation[2]

    return to_point

def rs2_project_point_to_pixel(intrin, point):
    pixel=[0.0 , 0.0]
    x = point[0] / point[2]
    y = point[1] / point[2]

    if intrin.model == 2 or intrin.model == 3:  # RS2_DISTORTION_MODIFIED_BROWN_CONRADY or RS2_DISTORTION_INVERSE_BROWN_CONRADY
        r2 = x * x + y * y
        f = 1 + intrin.coeffs[0] * r2 + intrin.coeffs[1] * r2 * r2 + intrin.coeffs[4] * r2 * r2 * r2
        x *= f
        y *= f
        dx = x + 2 * intrin.coeffs[2] * x * y + intrin.coeffs[3] * (r2 + 2 * x * x)
        dy = y + 2 * intrin.coeffs[3] * x * y + intrin.coeffs[2] * (r2 + 2 * y * y)
        x = dx
        y = dy

    if intrin.model == 4:  # RS2_DISTORTION_BROWN_CONRADY
        r2 = x * x + y * y
        f = 1 + intrin.coeffs[0] * r2 + intrin.coeffs[1] * r2 * r2 + intrin.coeffs[4] * r2 * r2 * r2
        xf = x * f
        yf = y * f
        dx = xf + 2 * intrin.coeffs[2] * x * y + intrin.coeffs[3] * (r2 + 2 * x * x)
        dy = yf + 2 * intrin.coeffs[3] * x * y + intrin.coeffs[2] * (r2 + 2 * y * y)
        x = dx
        y = dy
        

    if intrin.model == 5:  # RS2_DISTORTION_FTHETA
        r = np.sqrt(x * x + y * y)
        if r < np.finfo(float).eps:
            r = np.finfo(float).eps
        rd = 1.0 / intrin.coeffs[0] * np.arctan(2 * r * np.tan(intrin.coeffs[0] / 2.0))
        x *= rd / r
        y *= rd / r

    if intrin.model == 6:  # RS2_DISTORTION_KANNALA_BRANDT4
        r = np.sqrt(x * x + y * y)
        if r < np.finfo(float).eps:
            r = np.finfo(float).eps
        theta = np.arctan(r)
        theta2 = theta * theta
        series = 1 + theta2 * (intrin.coeffs[0] + theta2 * (intrin.coeffs[1] + theta2 * (intrin.coeffs[2] + theta2 * intrin.coeffs[3])))
        rd = theta * series
        x *= rd / r
        y *= rd / r

    pixel[0] = x * intrin.fx + intrin.ppx
    pixel[1] = y * intrin.fy + intrin.ppy

    return pixel

def rs2_fov(intrin):
    to_fov = [0.0, 0.0]
    to_fov[0] = (np.arctan2(intrin.ppx + 0.5, intrin.fx) + np.arctan2(intrin.width - (intrin.ppx + 0.5), intrin.fx)) * 57.2957795
    to_fov[1] = (np.arctan2(intrin.ppy + 0.5, intrin.fy) + np.arctan2(intrin.height - (intrin.ppy + 0.5), intrin.fy)) * 57.2957795
    return to_fov

def rs2_project_color_pixel_to_depth_pixel(data, depth_scale, depth_min, depth_max, depth_intrin, color_intrin, color_to_depth, depth_to_color, from_pixel):
    # print("Depth: ", data)
    start_pixel = [0.0, 0.0]
    to_pixel = [0.0, 0.0]
    min_point = np.zeros(3)
    min_transformed_point = np.zeros(3)

    # print(f"[orange1]Píxeles recibidos:{from_pixel} ")
    min_point = rs2_deproject_pixel_to_point(color_intrin, from_pixel, depth_min)
    min_transformed_point=rs2_transform_point_to_point(color_to_depth, min_point)
    start_pixel=rs2_project_point_to_pixel(depth_intrin, min_transformed_point)
    start_pixel=adjust_2D_point_to_boundary(start_pixel,depth_intrin.width, depth_intrin.height)

    end_pixel = [0.0, 0.0]
    max_point = np.zeros(3)
    max_transformed_point = np.zeros(3)

    max_point=rs2_deproject_pixel_to_point(color_intrin, from_pixel, depth_max)
    # print(f"[orange1]max point:{max_point}")   
    max_transformed_point=rs2_transform_point_to_point(color_to_depth, max_point)
    # print(f"[orange1]max_transformed_point:{max_transformed_point}")   
    end_pixel=rs2_project_point_to_pixel(depth_intrin, max_transformed_point)
    end_pixel=adjust_2D_point_to_boundary(end_pixel,depth_intrin.width, depth_intrin.height)

    min_dist = -1.0
    # print(f'[orange1]start px {start_pixel}, end px {end_pixel}')
    # Utiliza un bucle for para recorrer los valores en el rango deseado
    for x in np.arange(start_pixel[0], end_pixel[0], 0.1):
        for y in np.arange(start_pixel[1], end_pixel[1], 0.1):
            p = [x, y]
            # print(f"[orange1]p: {p}") 
            

            if not is_pixel_in_line(p, start_pixel, end_pixel):                
                break
            
           
            depth = depth_scale * data[int(p[1]), int(p[0])]
            # print(f"[orange1]depth: {depth}") 
            if depth == 0:
                continue

            # print(f"[orange1]depth: {depth}") 

            projected_pixel = [0.0, 0.0]
            point = np.zeros(3)
            transformed_point = np.zeros(3)

            # print(f"[orange1]p: {p}")  
            point = rs2_deproject_pixel_to_point(depth_intrin, p, depth)
            # print(f"[orange1]point:{point}")   

            transformed_point = rs2_transform_point_to_point(depth_to_color, point)
            # print(f"[orange1]point:{point}")   
            projected_pixel = rs2_project_point_to_pixel(color_intrin, transformed_point)
            
            
            new_dist = (projected_pixel[1] - from_pixel[1]) ** 2 + (projected_pixel[0] - from_pixel[0]) ** 2
            if new_dist < min_dist or min_dist < 0:
                min_dist = new_dist
                to_pixel[0] = p[0]
                to_pixel[1] = p[1]
            
    # print(f"[orange1]to pixel:{to_pixel}")   

    return to_pixel





