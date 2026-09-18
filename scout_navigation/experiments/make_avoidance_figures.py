"""Writes the obstacle-avoidance explainer diagrams as standalone English SVGs for slides."""
import pathlib

OUT = pathlib.Path(__file__).resolve().parent.parent / 'results' / 'figures'
INK, MUTED, ACCENT, SOFT, DANGER, DANGER_SOFT, BOX = (
    '#10202C', '#5D7284', '#0A6C8A', '#D9EBF2', '#A93B26', '#F6E0DA', '#F1F5F8')
FONT = "font-family='Helvetica, Arial, sans-serif' font-size='13'"


def svg(width, height, body):
    return (f"<svg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 {width} {height}' "
            f"width='{width}' height='{height}' {FONT}>\n"
            f"<defs><marker id='ar' viewBox='0 0 10 10' refX='9' refY='5' markerWidth='7' "
            f"markerHeight='7' orient='auto-start-reverse'><polygon points='0,1 10,5 0,9' fill='{INK}'/>"
            f"</marker></defs>\n<rect width='{width}' height='{height}' fill='white'/>\n{body}\n</svg>\n")


def text(x, y, s, fill=INK, anchor='middle', size=13, weight='normal'):
    s = s.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
    return (f"<text x='{x}' y='{y}' text-anchor='{anchor}' fill='{fill}' font-size='{size}' "
            f"font-weight='{weight}'>{s}</text>")


def arrow(x1, y1, x2, y2, both=False, stroke=INK, width=1.6):
    ends = "marker-start='url(#ar)' marker-end='url(#ar)'" if both else "marker-end='url(#ar)'"
    return f"<line x1='{x1}' y1='{y1}' x2='{x2}' y2='{y2}' stroke='{stroke}' stroke-width='{width}' {ends}/>"


def box(x, y, w, h, fill=BOX, stroke=INK):
    return f"<rect x='{x}' y='{y}' width='{w}' height='{h}' rx='3' fill='{fill}' stroke='{stroke}' stroke-width='1.4'/>"


def overview():
    b = [
        box(30, 112, 130, 66), text(95, 140, 'LiDAR scan', weight='bold'), text(95, 160, '720 beams · 360°', MUTED, size=12),
        arrow(160, 145, 196, 145),
        box(200, 100, 160, 90), text(280, 126, 'Three filters', weight='bold'),
        text(280, 146, 'ground · slopes', MUTED, size=12), text(280, 164, 'own towed carts', MUTED, size=12),
        arrow(360, 145, 396, 145),
        box(400, 112, 140, 66), text(470, 140, 'Obstacle points', weight='bold'), text(470, 160, 'world frame', MUTED, size=12),
        arrow(540, 145, 576, 145),
        box(580, 112, 150, 66), text(655, 140, 'Corridor test', weight='bold'), text(655, 160, 'blocking distance', MUTED, size=12),
        f"<path d='M 730 130 C 760 130 770 58 800 58' fill='none' stroke='{INK}' stroke-width='1.6' marker-end='url(#ar)'/>",
        arrow(730, 145, 800, 145),
        f"<path d='M 730 160 C 760 160 770 232 800 232' fill='none' stroke='{INK}' stroke-width='1.6' marker-end='url(#ar)'/>",
        text(762, 44, '< 1.16 m', DANGER, size=12), text(762, 138, '< 10 m', ACCENT, size=12), text(762, 256, 'clear', MUTED, size=12),
        box(804, 34, 166, 48, DANGER_SOFT, DANGER), text(887, 63, 'Stop and hold', DANGER, weight='bold'),
        box(804, 118, 166, 54, SOFT, ACCENT), text(887, 141, 'Plan a detour', ACCENT, weight='bold'),
        text(887, 160, 'splice into mission', ACCENT, size=12),
        box(804, 208, 166, 48), text(887, 237, 'Continue on line', weight='bold'),
        text(30, 288, 'Separately, before every spot turn: check the swing circle around the rover.', MUTED, 'start', 12),
    ]
    return svg(1000, 300, '\n'.join(b))


def perception():
    def panel(ox, title, verdict, arc, arc_w, label, ang, result):
        cx = ox + 150
        return [
            text(ox, 26, title, MUTED, 'start', 12, 'bold'),
            f"<path d='M {cx} 170 L {cx-120} 70 A 155 155 0 0 1 {cx+120} 70 Z' fill='{SOFT}' opacity='0.6'/>",
            text(cx, 124, 'scan field', MUTED, size=11),
            f"<path d='{arc}' fill='none' stroke='{DANGER}' stroke-width='{arc_w}'/>",
            text(cx, 60, label, DANGER, size=12),
            f"<circle cx='{cx}' cy='170' r='9' fill='{INK}'/>", text(cx, 196, 'rover', MUTED, size=11),
            arrow(*ang, both=True, width=1.2),
            text(cx, 230, verdict, DANGER, weight='bold'), text(cx, 262, result, MUTED, size=12),
        ]
    a = panel(30, 'A · Returns from a slope → discard',
              'angular width > 90°', 'M 62 96 Q 180 40 298 96', 4, 'returns form one wide band', (62, 142, 298, 142), '→ classified as terrain')
    b = panel(560, 'B · Returns from an object → keep',
              'angular width < 90°', 'M 686 78 Q 710 72 734 78', 5, 'returns in one narrow cluster', (686, 142, 734, 142), '→ classified as obstacle')
    dashes = [f"<line x1='{x}' y1='{y}' x2='{x}' y2='132' stroke='{DANGER}' stroke-width='1.5' stroke-dasharray='3 3'/>"
              for x, y in ((62, 96), (298, 96), (686, 78), (734, 78))]
    sep = f"<line x1='500' y1='40' x2='500' y2='270' stroke='#C9D6E0'/>"
    return svg(1000, 290, '\n'.join(a + b + dashes + [sep]))


def corridor():
    b = [
        f"<rect x='70' y='98' width='860' height='64' fill='{SOFT}'/>",
        f"<line x1='70' y1='130' x2='930' y2='130' stroke='{ACCENT}' stroke-width='2' stroke-dasharray='8 5'/>",
        f"<line x1='70' y1='98' x2='930' y2='98' stroke='{ACCENT}'/>", f"<line x1='70' y1='162' x2='930' y2='162' stroke='{ACCENT}'/>",
        text(78, 90, 'corridor edge  +0.85 m', ACCENT, 'start', 12), text(78, 180, 'corridor edge  −0.85 m', ACCENT, 'start', 12),
        text(880, 122, 'survey line', ACCENT, size=12),
        f"<rect x='150' y='118' width='26' height='24' rx='2' fill='{INK}'/>", text(163, 212, 'rover', MUTED, size=11),
        f"<circle cx='560' cy='122' r='17' fill='{DANGER}'/>", text(560, 76, 'inside corridor → blocking', DANGER, weight='bold'),
        f"<circle cx='740' cy='205' r='17' fill='{DANGER}' opacity='0.35'/>", text(740, 240, 'outside corridor → ignored', MUTED),
        arrow(176, 130, 543, 130, width=1.4), text(360, 150, 'blocking distance (measured along the path)'),
    ]
    return svg(1000, 250, '\n'.join(b))


def detour():
    b = [
        f"<line x1='60' y1='215' x2='950' y2='215' stroke='{ACCENT}' stroke-width='1.6' stroke-dasharray='8 5'/>",
        text(900, 236, 'survey line', ACCENT, size=12),
        f"<rect x='470' y='96' width='70' height='52' rx='3' fill='{DANGER}'/>", text(505, 86, 'obstacle', DANGER, weight='bold'),
        arrow(470, 160, 540, 160, both=True, stroke=DANGER, width=1.2), text(505, 178, 'measured return extent', DANGER, size=12),
        f"<line x1='470' y1='184' x2='470' y2='202' stroke='{DANGER}' stroke-dasharray='3 3'/>",
        f"<line x1='540' y1='184' x2='540' y2='202' stroke='{DANGER}' stroke-dasharray='3 3'/>",
        arrow(400, 194, 470, 194, both=True, width=1.2), arrow(540, 194, 610, 194, both=True, width=1.2),
        text(435, 210, 'margin 1.0 m', MUTED, size=12), text(575, 210, 'margin 1.0 m', MUTED, size=12),
        f"<path d='M 90 215 L 190 215 C 280 215 310 130 400 130 L 610 130 C 700 130 740 215 830 215 L 950 215' fill='none' stroke='{ACCENT}' stroke-width='3.5'/>",
        f"<rect x='78' y='203' width='24' height='22' rx='2' fill='{INK}'/>", text(90, 246, 'rover', MUTED, size=11),
        arrow(190, 256, 400, 256, both=True, width=1.2), text(295, 276, '1  ramp out', weight='bold'),
        arrow(400, 256, 610, 256, both=True, width=1.2), text(505, 276, '2  hold beside', weight='bold'),
        arrow(610, 256, 830, 256, both=True, width=1.2), text(720, 276, '3  ramp back', weight='bold'),
        arrow(360, 130, 360, 215, both=True, stroke=ACCENT, width=1.2), text(348, 176, 'lateral offset', ACCENT, 'end', 12),
        text(295, 300, 'L = √(6·D·v² / a_lat)', MUTED, size=12), text(720, 300, 'each ramp sized from its own shift', MUTED, size=12),
    ]
    return svg(1000, 320, '\n'.join(b))


def side():
    b = [
        f"<line x1='60' y1='150' x2='940' y2='150' stroke='{ACCENT}' stroke-width='1.6' stroke-dasharray='8 5'/>",
        text(890, 171, 'survey line', ACCENT, size=12),
        f"<rect x='470' y='108' width='64' height='44' rx='3' fill='{DANGER}'/>",
        f"<line x1='600' y1='60' x2='900' y2='60' stroke='{ACCENT}' stroke-width='3'/>",
        text(610, 50, 'left detour needs +2.95 m  ← chosen', ACCENT, 'start', 12, 'bold'),
        f"<line x1='600' y1='214' x2='900' y2='214' stroke='{ACCENT}' stroke-width='3' opacity='0.35'/>",
        text(610, 236, 'right detour needs −1.75 m', MUTED, 'start', 12),
        f"<rect x='150' y='98' width='24' height='22' rx='2' fill='{INK}'/>",
        arrow(162, 120, 162, 150, both=True, width=1.2), text(180, 112, 'rover currently at +1.0 m', INK, 'start', 12),
        text(150, 200, 'from +1.0: left needs 1.95 m of movement, right needs 2.75 m and crosses the line', MUTED, 'start', 12),
        text(150, 222, 'rule: pick the side nearer to where the rover already is', INK, 'start', 12, 'bold'),
    ]
    return svg(1000, 250, '\n'.join(b))


def swing():
    b = [
        f"<line x1='60' y1='150' x2='700' y2='150' stroke='{ACCENT}' stroke-width='1.6' stroke-dasharray='8 5'/>",
        text(90, 140, 'this line', ACCENT, 'start', 12),
        f"<line x1='700' y1='150' x2='700' y2='40' stroke='{ACCENT}' stroke-width='1.6' stroke-dasharray='8 5'/>",
        text(712, 60, 'next line', ACCENT, 'start', 12),
        f"<circle cx='700' cy='150' r='78' fill='none' stroke='{INK}' stroke-width='1.5' stroke-dasharray='5 4'/>",
        f"<rect x='686' y='138' width='28' height='24' rx='2' fill='{INK}'/>",
        f"<line x1='700' y1='150' x2='778' y2='150' stroke='{INK}' stroke-width='1.2'/>", text(740, 142, '1.08 m', size=12),
        text(620, 240, 'swing circle of the body', MUTED, size=12),
        f"<circle cx='700' cy='215' r='6' fill='{DANGER}'/>",
        text(716, 219, 'pole 0.90 m away → inside circle → hold', DANGER, 'start', 12, 'bold'),
        f"<circle cx='860' cy='150' r='17' fill='{DANGER}' opacity='0.35'/>", text(884, 154, 'outside → turn allowed', MUTED, 'start', 12),
    ]
    return svg(1000, 250, '\n'.join(b))


FIGURES = {
    '1_overview': overview, '2_perception_slope_vs_object': perception, '3_corridor': corridor,
    '4_detour_path': detour, '5_side_selection': side, '6_swing_circle': swing,
}

if __name__ == '__main__':
    OUT.mkdir(parents=True, exist_ok=True)
    for name, make in FIGURES.items():
        (OUT / f'{name}.svg').write_text(make(), encoding='utf-8')
        print(f'wrote {OUT / name}.svg')
