import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, Circle

plt.rcParams["mathtext.fontset"] = "dejavusans"

fig, ax = plt.subplots(figsize=(15.5, 8.6))
ax.set_xlim(0, 125); ax.set_ylim(0, 100); ax.axis("off")

NAVY=("#1f3b5c","white"); ORANGE=("#e08a3c","white"); GREEN=("#2e7d32","white")
PLANT=("#2f5f8a","#cfe0f3"); REF=("#000000","white")

def box(cx,cy,w,h,text,style,fs=13,bold=False):
    ec,fc=style
    ax.add_patch(Rectangle((cx-w/2,cy-h/2),w,h,lw=2.0,edgecolor=ec,facecolor=fc,zorder=2))
    ax.text(cx,cy,text,ha="center",va="center",fontsize=fs,fontweight=("bold" if bold else "normal"),zorder=3)
    return dict(cx=cx,cy=cy,L=(cx-w/2,cy),R=(cx+w/2,cy),T=(cx,cy+h/2),B=(cx,cy-h/2))

def summ(cx,cy,r=2.9,signs=None,sym="sum"):
    ax.add_patch(Circle((cx,cy),r,lw=2.0,edgecolor="#000",facecolor="white",zorder=3))
    d=r*0.72
    if sym=="mult":
        ax.text(cx,cy,"×",ha="center",va="center",fontsize=16,zorder=4)
    else:
        ax.plot([cx-d,cx+d],[cy-d,cy+d],color="#000",lw=1.4,zorder=4)
        ax.plot([cx-d,cx+d],[cy+d,cy-d],color="#000",lw=1.4,zorder=4)
    if signs:
        o=r+1.8
        pos={"left":(cx-o,cy+1.7),"bottom":(cx-2.1,cy-o-0.3),"top":(cx-2.1,cy+o),"topr":(cx+o,cy+1.7)}
        for k,s in signs.items(): ax.text(*pos[k],s,ha="center",va="center",fontsize=16,fontweight="bold",zorder=4)
    return dict(cx=cx,cy=cy,L=(cx-r,cy),R=(cx+r,cy),T=(cx,cy+r),B=(cx,cy-r))

def line(pts,lw=2.0): ax.plot([p[0] for p in pts],[p[1] for p in pts],color="#000",lw=lw,zorder=1,solid_capstyle="round")
def arrow(p,q,lw=2.0): ax.annotate("",xy=q,xytext=p,arrowprops=dict(arrowstyle="-|>",lw=lw,color="#000"),zorder=1)
def lbl(x,y,t,fs=12,**k): ax.text(x,y,t,fontsize=fs,**k)
def dot(x,y): ax.add_patch(Circle((x,y),0.7,color="#000",zorder=4))

# ===== forward =====
ref = box(11,70,16,11,r"$\Delta\phi_{ref}=0$",REF,bold=True); lbl(11,62.5,"(level plate)",fs=10,ha="center",color="#555")
cmp_= summ(28,70,signs={"left":"+","bottom":"−"})
Kp  = box(47,81,14,10,r"$K_p$",ORANGE,fs=15,bold=True); lbl(47,75.2,"proportional",fs=9.5,ha="center",color="#555")
Ki  = box(47,59,14,10,r"$K_i\!\int dt$",ORANGE,fs=14,bold=True); lbl(47,53.2,"integral",fs=9.5,ha="center",color="#555")
psum= summ(64,70,signs={"top":"+","bottom":"+"})
plant=box(89,70,26,15,"Dual-servo\ncrank-slider exciter\n(Plant)",PLANT,bold=True)
u0  = box(89,91,22,10,r"Base speed  $u_0$" "\n(BASE_OFFSET)",REF)

arrow(ref["R"],cmp_["L"])
line([cmp_["R"],(38,70)]); dot(38,70); lbl(35,72.5,r"$e$",fs=14,ha="center")
line([(38,70),(38,81)]); arrow((38,81),Kp["L"])
line([(38,70),(38,59)]); arrow((38,59),Ki["L"])
line([Kp["R"],(64,81)]); arrow((64,81),psum["T"])
line([Ki["R"],(64,59)]); arrow((64,59),psum["B"])
arrow(psum["R"],plant["L"]); lbl(70,72.5,r"$\Delta$",fs=14,ha="center")
arrow(u0["B"],plant["T"])
arrow(plant["R"],(112,70))
lbl(113.5,70,r"$\ddot z$  harmonic" "\n" "excitation\n" r"($\approx$2 Hz)",fs=12.5,ha="left",va="center",fontweight="bold",color="#1a5d1a")

# ===== feedback (detailed phase detector) =====
imu = box(89,32,23,13,"IMU (MPU-6500)\n" r"gyro $\omega_{roll}$ + accel $a_z$",GREEN,fs=12,bold=True)
hp  = box(63,32,17,13,"High-pass\nremove\ngravity / DC",NAVY,fs=12)
mult= summ(44,32,r=3.4,sym="mult")
lp  = box(24,32,16,13,"Low-pass\n" r"$\langle\,\cdot\,\rangle$",NAVY,fs=12)

arrow(plant["B"],imu["T"]); lbl(91,50,r"plate roll $\theta$",fs=12,ha="left",va="center")
arrow(imu["L"],hp["R"]); lbl(76.5,35.5,r"$\omega_{roll},\,a_z$",fs=11,ha="center")
arrow(hp["L"],mult["R"]); lbl(53,35.5,r"$\tilde\omega,\,\tilde a$",fs=11,ha="center")
arrow(mult["L"],lp["R"])
lbl(44,24,"multiply",fs=9.5,ha="center",color="#555")
line([lp["T"],(24,63),(28,63)]); arrow((28,63),cmp_["B"])
lbl(20,50,r"$\langle\tilde\omega\,\tilde a\rangle \propto \sin\Delta\phi$" "\n(phase error)",fs=11,ha="center",va="center")

ax.text(58,7,"IMU-Based Phase-Synchronization Control of a Dual-Servo Harmonic Exciter",
        ha="center",fontsize=15,fontweight="bold")

plt.tight_layout()
out=r"c:\Users\chang\OneDrive\문서\PlatformIO\Projects\servo_test\block_diagram4.png"
plt.savefig(out,dpi=200,bbox_inches="tight",facecolor="white")
print("saved",out)
