import xml.etree.ElementTree as ET
import re
import ezdxf
import numpy as np
from svgpathtools import Path as SVGPath, Line as SVGLine, QuadraticBezier, CubicBezier

def svg_to_dxf_final(input_svg, output_dxf, tolerance=2.0, debug=True):
    """
    SVG to DXF - 最终版本
    """
    
    # 解析SVG
    tree = ET.parse(input_svg)
    root = tree.getroot()
    
    ns = {'svg': 'http://www.w3.org/2000/svg'}
    
    # 收集所有path
    paths_data = []
    
    for g in root.findall('.//{http://www.w3.org/2000/svg}g', ns):
        g_transform = parse_transform(g.get('transform', ''))
        
        for path in g.findall('{http://www.w3.org/2000/svg}path'):
            path_d = path.get('d', '')
            if path_d:
                paths_data.append({
                    'd': path_d,
                    'transform': g_transform
                })
    
    # 根目录下的path
    for path in root.findall('{http://www.w3.org/2000/svg}path'):
        path_d = path.get('d', '')
        if path_d:
            paths_data.append({
                'd': path_d,
                'transform': None
            })
    
    if debug:
        print(f"发现 {len(paths_data)} 个路径")
    
    if not paths_data:
        print("错误：未找到路径")
        return
    
    # 计算边界
    all_coords = []
    for pd in paths_data:
        try:
            path = SVGPath(pd['d'])
            transform = pd['transform']
            
            for segment in path:
                for pt in [segment.start, segment.end]:
                    if transform:
                        pt = apply_transform(pt, transform)
                    all_coords.append((pt.real, pt.imag))
        except:
            continue
    
    if not all_coords:
        print("错误：无坐标")
        return
    
    x_coords = [c[0] for c in all_coords]
    y_coords = [c[1] for c in all_coords]
    
    min_x, max_x = min(x_coords), max(x_coords)
    min_y, max_y = min(y_coords), max(y_coords)
    
    if debug:
        print(f"边界: X=[{min_x:.1f}, {max_x:.1f}], Y=[{min_y:.1f}, {max_y:.1f}]")
    
    # 缩放
    svg_width = max_x - min_x
    svg_height = max_y - min_y
    scale = 180 / max(svg_width, svg_height) if max(svg_width, svg_height) > 0 else 1
    
    center_x = (min_x + max_x) / 2
    center_y = (min_y + max_y) / 2
    
    def final_transform(pt):
        x = (pt.real - center_x) * scale
        y = (pt.imag - center_y) * scale
        return (round(x, 2), round(y, 2))
    
    doc = ezdxf.new('R2010')
    msp = doc.modelspace()
    
    polyline_count = 0
    
    # 处理每个路径
    for pd in paths_data:
        try:
            path = SVGPath(pd['d'])
            transform = pd['transform']
            
            points = []
            
            for segment in path:
                # 获取起点
                start = segment.start
                if transform:
                    start = apply_transform(start, transform)
                
                if len(points) == 0:
                    points.append(final_transform(start))
                
                # 处理各类曲线
                if isinstance(segment, SVGLine):
                    end = segment.end
                    if transform:
                        end = apply_transform(end, transform)
                    points.append(final_transform(end))
                    
                elif isinstance(segment, CubicBezier):
                    # 三次贝塞尔：获取控制点和终点
                    c1 = segment.control1
                    c2 = segment.control2
                    end = segment.end
                    
                    if transform:
                        c1 = apply_transform(c1, transform)
                        c2 = apply_transform(c2, transform)
                        end = apply_transform(end, transform)
                    
                    # 从起点开始离散化
                    curve_pts = discretize_cubic_v2(start, c1, c2, end, tolerance)
                    for pt in curve_pts[1:]:  # 跳过第一个（已存在）
                        points.append(final_transform(pt))
                        
                elif isinstance(segment, QuadraticBezier):
                    # 二次贝塞尔
                    c = segment.control
                    end = segment.end
                    
                    if transform:
                        c = apply_transform(c, transform)
                        end = apply_transform(end, transform)
                    
                    # 转三次贝塞尔
                    cp1 = start + (c - start) * 2/3
                    cp2 = end + (c - end) * 2/3
                    
                    curve_pts = discretize_cubic_v2(start, cp1, cp2, end, tolerance)
                    for pt in curve_pts[1:]:
                        points.append(final_transform(pt))
                else:
                    # 其他类型
                    end = segment.end
                    if transform:
                        end = apply_transform(end, transform)
                    points.append(final_transform(end))
            
            # 写入DXF
            if len(points) >= 2:
                # 检查闭合
                first = complex(points[0][0], points[0][1])
                last = complex(points[-1][0], points[-1][1])
                is_closed = abs(last - first) < 3.0
                
                if is_closed and len(points) > 2:
                    points = points[:-1]
                
                msp.add_lwpolyline(points, close=is_closed)
                polyline_count += 1
                
        except Exception as e:
            if debug:
                print(f"  错误: {e}")
            continue
    
    doc.saveas(output_dxf)
    print(f"\n✓ 转换完成: {output_dxf}")
    print(f"  共 {polyline_count} 条多段线")


def parse_transform(transform_str):
    """解析SVG transform属性"""
    if not transform_str:
        return None
    
    match = re.search(r'matrix\(([^)]+)\)', transform_str)
    if match:
        values = [float(x) for x in match.group(1).split()]
        return {
            'a': values[0], 'b': values[1], 
            'c': values[2], 'd': values[3],
            'e': values[4], 'f': values[5]
        }
    return None


def apply_transform(pt, transform):
    """应用变换矩阵"""
    if not transform:
        return pt
    
    x = transform['a'] * pt.real + transform['c'] * pt.imag + transform['e']
    y = transform['b'] * pt.real + transform['d'] * pt.imag + transform['f']
    
    return complex(x, y)


def discretize_cubic_v2(start, control1, control2, end, tolerance):
    """离散化三次贝塞尔 - 修复版"""
    points = [start]
    
    # 计算曲线长度
    length = abs(end - start)
    num_samples = max(3, int(length / tolerance) + 1)
    
    for i in range(1, num_samples + 1):
        t = i / num_samples
        
        # 三次贝塞尔公式
        t2 = t * t
        t3 = t2 * t
        mt = 1 - t
        mt2 = mt * mt
        mt3 = mt2 * mt
        
        x = mt3*start.real + 3*mt2*t*control1.real + 3*mt*t2*control2.real + t3*end.real
        y = mt3*start.imag + 3*mt2*t*control1.imag + 3*mt*t2*control2.imag + t3*end.imag
        
        points.append(complex(x, y))
    
    return points


# 使用
if __name__ == "__main__":
    svg_to_dxf_final(
        input_svg='input.svg',
        output_dxf='output.dxf',
        tolerance=2.0,
        debug=True
    )
