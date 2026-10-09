"""Render Q4 coverage geometry from the retained simulator's waypoint layout.

This is a plotting companion to the MATLAB geometry script. It computes the
existing sampled angular-gap diagnostic without modifying the model. ReportLab
draws the figure; Poppler rasterizes a temporary vector page to PNG.
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile

from reportlab.graphics import renderPDF
from reportlab.graphics.shapes import Circle, Drawing, Line, Rect, String
from reportlab.lib.colors import HexColor, white

import q4_directional_coverage as model


def make_figure():
    points = model._build_q4_waypoints()
    gap, worst, _ = model.angle_gap_check(points)
    margin = 180.0 - gap
    drawing = Drawing(840, 810)
    drawing.add(Rect(0, 0, 840, 810, fillColor=white, strokeColor=None))
    blue, orange, green, red = map(HexColor, ['#26669c', '#cc7722', '#258752', '#c03535'])
    grey = HexColor('#70777e')
    # One scale in both directions preserves the geometry.
    scale, origin_x, origin_y = 0.115, 345, 425

    def xy(point):
        return origin_x + scale * point[0], origin_y + scale * point[1]

    def text(x, y, label, size=11, color=grey, anchor='start'):
        drawing.add(String(x, y, label, fontName='Helvetica', fontSize=size,
                           fillColor=color, textAnchor=anchor))

    text(420, 778, 'Directional-source coverage: 24-waypoint layout', 18, blue, 'middle')
    text(420, 752, f'Sampled maximum angular gap: {gap:.3f} deg   |   Margin to 180 deg: {margin:.3f} deg',
         12, red, 'middle')

    for value in range(-2000, 2501, 500):
        x, _ = xy((value, 0))
        drawing.add(Line(x, 149, x, 701, strokeColor=HexColor('#e8ebef'), strokeWidth=0.5))
        text(x, 133, str(value), 9, anchor='middle')
    for value in range(-2000, 2001, 500):
        _, y = xy((0, value))
        drawing.add(Line(69, y, 690, y, strokeColor=HexColor('#e8ebef'), strokeWidth=0.5))
        text(60, y-3, str(value), 9, anchor='end')
    drawing.add(Rect(69, 149, 621, 552, fillColor=None, strokeColor=grey, strokeWidth=0.7))
    text(379, 113, 'x (m)', 12, anchor='middle')
    text(35, 716, 'y (m)', 12)
    drawing.add(Circle(origin_x, origin_y, scale*model.ARENA_R,
                       fillColor=HexColor('#f1f5fa'), strokeColor=blue, strokeWidth=1.4))
    wx, wy = xy(worst)
    drawing.add(Circle(wx, wy, scale*model.R_RECV_MIN, fillColor=None,
                       strokeColor=orange, strokeWidth=1.1, strokeDashArray=[5, 3]))
    for point in points:
        if model.dist(point, worst) <= model.R_RECV_MIN + 1e-9:
            px, py = xy(point)
            drawing.add(Line(wx, wy, px, py, strokeColor=grey, strokeWidth=0.8,
                             strokeDashArray=[2, 3]))
    for index, point in enumerate(points):
        x, y = xy(point)
        if index == 0:
            drawing.add(Circle(x, y, 5, fillColor=green, strokeColor=white))
        elif index <= model.Q4_INNER_N:
            drawing.add(Circle(x, y, 4, fillColor=blue, strokeColor=white))
        else:
            drawing.add(Rect(x-4, y-4, 8, 8, fillColor=orange, strokeColor=white))
    drawing.add(Line(wx-6, wy-6, wx+6, wy+6, strokeColor=red, strokeWidth=2))
    drawing.add(Line(wx-6, wy+6, wx+6, wy-6, strokeColor=red, strokeWidth=2))
    text(wx+10, wy-18, f'Worst sampled point ({worst[0]:.0f}, {worst[1]:.0f}) m', 10, red)

    labels=[('Target region: radius 1800 m',blue),
            ('10 inner waypoints: radius 1000 m',blue),
            ('13 outer waypoints: radius 1880 m',orange),
            ('Center waypoint',green),
            ('Reception disk at sampled worst point',orange),
            ('Worst sampled point',red)]
    for index,(label,color) in enumerate(labels):
        x=80+(index%2)*355; y=86-(index//2)*20
        drawing.add(Line(x,y+3,x+15,y+3,strokeColor=color,strokeWidth=2))
        text(x+22,y,label,10,color)
    text(420,15,'Geometric diagnostic; not a measured robot trajectory.',10,anchor='middle')
    return drawing, gap, margin


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,
                        default=Path(__file__).resolve().parents[1]/'results/generated/figures/q4_directional_coverage.png')
    parser.add_argument('--renderer',default=shutil.which('pdftoppm'),
                        help='Path to Poppler pdftoppm if it is not on PATH')
    args=parser.parse_args()
    if not args.renderer:
        parser.error('Poppler pdftoppm is required for PNG export; supply --renderer or add it to PATH.')
    drawing,gap,margin=make_figure()
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with tempfile.TemporaryDirectory(prefix='q4_geometry_') as tmp:
        pdf=Path(tmp)/'geometry.pdf';prefix=Path(tmp)/'geometry'
        renderPDF.drawToFile(drawing,str(pdf))
        subprocess.run([args.renderer,'-png','-singlefile','-r','180',str(pdf),str(prefix)],check=True)
        shutil.copyfile(prefix.with_suffix('.png'),args.output)
    print(f'Wrote {args.output}; sampled gap={gap:.6f} deg, margin={margin:.6f} deg')


if __name__=='__main__':
    main()
