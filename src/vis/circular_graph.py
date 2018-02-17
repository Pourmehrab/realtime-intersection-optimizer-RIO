import numpy as np
import os
import random


def add_nodes(f, freq, angle_step, r, n_name):
    f.write('''\\foreach \\t in {{0,...,{:d}}}
    	\\node ({:s}\\t) at (-90+\\t*{:2.2f} :{:2.2f}) [circle,inner sep=0pt,minimum size=1mm] {{}};\n'''.format(
        freq - 1, n_name, angle_step, r))


def connect(f, freq, angle_step, n_from, n_to, l_keep, l_corr_l, l_corr_r, mode):
    if mode == 'all':
        f.write('''\\foreach \\t in {{0,...,{:d}}}
            \\draw[->,> = latex] ({:s}\\t) to [out=-30+90+\\t*{:2.2f},in=30-90+\\t*{:2.2f}]({:s}\\t);\n'''.format(
            freq - 1, n_from, angle_step, angle_step, n_to))
    elif mode == 'none':
        str_nodes = str(l_keep + l_corr_l + l_corr_r).strip('[]')
        f.write('''\\foreach \\t in {{{:s}}}
                    \\draw[->,> = latex] ({:s}\\t) to [out=-30+90+\\t*{:2.2f},in=30-90+\\t*{:2.2f}]({:s}\\t);\n'''.format(
            str_nodes, n_from, angle_step, angle_step, n_to))
    else:
        str_nodes = str(l_keep).strip('[]')
        f.write('''\\foreach \\t in {{{:s}}}
                    \\draw[->,> = latex] ({:s}\\t) to [out=-30+90+\\t*{:2.2f},in=30-90+\\t*{:2.2f}]({:s}\\t);\n'''.format(
            str_nodes, n_from, angle_step, angle_step, n_to))

        for i in range(len(l_corr_l)):
            f.write(
                '''\\draw[->,> = latex,thick] ({:s}{:d}) to [out=90+{:d}*{:2.2f},in=-90+{:d}*{:2.2f}]({:s}{:d});\n'''.format(
                    n_from, l_corr_l[i], l_corr_l[i], angle_step, l_corr_r[i], angle_step, n_to, l_corr_r[i]))


filepath_in = os.path.join('circular_in.txt')
if os.path.exists(filepath_in):
    f_in = open(filepath_in, 'w')
else:
    f_in = open(filepath_in, 'x')
filepath_out = os.path.join('circular_out.txt')
if os.path.exists(filepath_out):
    f_out = open(filepath_out, 'w')
else:
    f_out = open(filepath_out, 'x')

f_in.write('''\\begin{tikzpicture}
\\usetikzlibrary{decorations, decorations.text,}\n''')
f_out.write('''\\begin{tikzpicture}
\\usetikzlibrary{decorations, decorations.text,}\n''')
diameter = 1.5  # cm

f_in.write(
    '\\node (core) at ({:2.2f},{:2.2f}) [circle,fill=gray!50!white,draw,inner sep=0pt,minimum size={:2.2f}cm,line width=1mm] {{\\tiny Optimize}};\n'.format(
        0, 0, diameter))
f_out.write(
    '\\node (core) at ({:2.2f},{:2.2f}) [circle,fill=gray!50!white,draw,inner sep=0pt,minimum size={:2.2f}cm,line width=1mm] {{\\tiny Optimize}};\n'.format(
        0, 0, diameter))

freq = 5 * 20  # Hertz (per sec) pick even
angle_step = 360 / freq
angles = [i * angle_step for i in range(freq)]

left_nodes = [i for i in range(freq // 2)]
random.shuffle(left_nodes)
right_nodes = [i for i in range(freq // 2, freq)]
random.shuffle(right_nodes)

l_keep, l_corr_l, l_corr_r = [], [], []

for i in range(freq // 6 + 1):
    l_corr_l.append(left_nodes[-1 * (i + 1)])
    l_corr_r.append(right_nodes[i])

    l_keep.append(left_nodes[i])
    l_keep.append(right_nodes[-1 * (i + 1)])
l_corr_l.sort()
l_corr_r.sort(reverse=True)

gap = 0.6  # cm

add_nodes(f_in, freq, angle_step, diameter / 2, 'o')

# two half sectors
a1, r1, a2, r2 = -90, diameter / 2 + gap, 90, diameter / 2 + 2 * gap
f_in.write('''\\foreach \m in {{0,1}}{{
\draw [fill=gray!50!white,rounded corners,line width=1mm,rotate around={{180*\m:(0,0)}}] 
    ({:2.2f}:{:2.2f}) arc ({:2.2f}:{:2.2f}:{:2.2f}) -- 
    ({:2.2f}:{:2.2f}) arc ({:2.2f}: {:2.2f}:{:2.2f}) -- cycle;
}}\n'''.format(a1, r2, a1, a2, r2, a2, r1, a2, a1, r1))
add_nodes(f_in, freq, angle_step, r1, 'l1')

add_nodes(f_in, freq, angle_step, r2, 'l2')

r3, r4, r5, r6, r7 = r2 + gap, r2 + 2 * gap, r2 + 3 * gap, r2 + 4 * gap, r2 + 5 * gap
f_in.write('''\draw[even odd rule,fill=gray!50!white,line width=1mm]
(0,0) circle ({:2.2f}cm)
(0,0) circle ({:2.2f}cm);
\draw[line width=1mm]
(0,0) circle ({:2.2f}cm);\n'''.format(r3, r4, r5, r6))
f_out.write('''\draw[line width=1mm]
(0,0) circle ({:2.2f}cm);\n'''.format(r5))
add_nodes(f_in, freq, angle_step, r3, 'l3')
add_nodes(f_in, freq, angle_step, r4, 'l4')
add_nodes(f_in, freq, angle_step, r5, 'l5')
add_nodes(f_out, freq, angle_step, r5, 'l5')

# make arcs
connect(f_in, freq, angle_step, 'l1', 'o', l_keep, l_corr_l, l_corr_r, 'none')
connect(f_in, freq, angle_step, 'l3', 'l2', l_keep, l_corr_l, l_corr_r, 'none')
connect(f_in, freq, angle_step, 'l5', 'l4', l_keep, l_corr_l, l_corr_r, 'all')
connect(f_out, freq, angle_step, 'o', 'l5', l_keep, l_corr_l, l_corr_r, 'none')

# add text
f_in.write(
    '\\draw[rotate=-45, postaction={{decorate, decoration={{text along path, raise=4pt, text align={{align=center}}, text={{|\\scriptsize|{:s}}}, reverse path}}}}] (0,0) circle ({:2.2f}cm);\n'.format(
        'Problemize', diameter / 2))
f_in.write(
    '\\draw[rotate=-45, postaction={{decorate, decoration={{text along path, raise=4pt, text align={{align=center}}, text={{|\\scriptsize|{:s}}}, reverse path}}}}] (0,0) circle ({:2.2f}cm);\n'.format(
        'Correct Perception', r1))
f_in.write(
    '\\draw[rotate=-135,postaction={{decorate, decoration={{text along path, raise=4pt, text align={{align=center}}, text={{|\\scriptsize|{:s}}}, reverse path}}}}] (0,0) circle ({:2.2f}cm);\n'.format(
        'Keep Perception', r1))
f_in.write(
    '\\draw[rotate=-45, postaction={{decorate, decoration={{text along path, raise=4pt, text align={{align=center}}, text={{|\\scriptsize|{:s}}}, reverse path}}}}] (0,0) circle ({:2.2f}cm);\n'.format(
        'Abstractify', r2))
f_in.write(
    '\\draw[rotate=-45, postaction={{decorate, decoration={{text along path, raise=4pt, text align={{align=center}}, text={{|\\scriptsize|{:s}}}, reverse path}}}}] (0,0) circle ({:2.2f}cm);\n'.format(
        'Communicate', r3))
f_in.write(
    '\\draw[rotate=-45, postaction={{decorate, decoration={{text along path, raise=4pt, text align={{align=center}}, text={{|\\scriptsize|{:s}}}, reverse path}}}}] (0,0) circle ({:2.2f}cm);\n'.format(
        'Sence', r4))
f_in.write(
    '\\draw[rotate=-45, postaction={{decorate, decoration={{text along path, raise=13pt, text align={{align=center}}, text={{{:s}}}, reverse path}}}}] (0,0) circle ({:2.2f}cm);\n'.format(
        'REAL-WORLD', r5))
f_out.write(
    '\\draw[rotate=-45, postaction={{decorate, decoration={{text along path, raise=13pt, text align={{align=center}}, text={{{:s}}}, reverse path}}}}] (0,0) circle ({:2.2f}cm);\n'.format(
        'REAL-WORLD', r5))
f_in.write('''\\draw [->,> = latex,very thick] 
    (45:{:2.2f}) arc (45:70:{:2.2f});
\\draw[white,rotate=-125, postaction={{decorate, decoration={{text along path, raise=4pt, text align={{align=center}}, text={{Time}}, reverse path}}}}] (0,0) circle ({:2.2f}cm);\n'''.format(
    r5 + gap, r5 + gap, r5 + gap))
f_out.write('''\\draw [->,> = latex,very thick] 
    (45:{:2.2f}) arc (45:70:{:2.2f});
\\draw[white,rotate=-125, postaction={{decorate, decoration={{text along path, raise=4pt, text align={{align=center}}, text={{Time}}, reverse path}}}}] (0,0) circle ({:2.2f}cm);\n'''.format(
    r5 + gap, r5 + gap, r5 + gap))

f_in.write('\\end{tikzpicture}')
f_out.write('\\end{tikzpicture}')
f_in.close()
f_out.close()
