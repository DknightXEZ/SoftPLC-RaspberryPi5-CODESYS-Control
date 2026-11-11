import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.patches import FancyBboxPatch
fp='/usr/share/fonts/opentype/tlwg/Loma.otf'
font_manager.fontManager.addfont(fp); fb='/usr/share/fonts/opentype/tlwg/Loma-Bold.otf'; font_manager.fontManager.addfont(fb)
plt.rcParams['font.family']=[font_manager.FontProperties(fname=fp).get_name(),'DejaVu Sans']
W,H=1920,1000
fig=plt.figure(figsize=(19.2,10),dpi=150); ax=fig.add_axes([0,0,1,1]); ax.set_xlim(0,W); ax.set_ylim(H,0); ax.axis('off')
BLUE='#2f6db5'; GOLD='#c98a00'; RED='#c0392b'; GREEN='#27ae60'
def box(x,y,w,h,title,lines=(),fc='#fff6dc',ec=GOLD,ts=22,ls=17):
    ax.add_patch(FancyBboxPatch((x,y),w,h,boxstyle='round,pad=0,rounding_size=14',fc=fc,ec=ec,lw=2.5))
    n=len(lines); 
    ty=y+h/2-(n*ls*1.45)/2 if n else y+h/2
    ax.text(x+w/2,ty,title,ha='center',va='center',fontsize=ts,fontweight='bold')
    for i,l in enumerate(lines):
        ax.text(x+w/2,ty+ (i+1)*ls*1.75,l,ha='center',va='center',fontsize=ls)
def wire(pts,c,label=None,lp=None,lw=3,arrow=True):
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
    ax.plot(xs,ys,color=c,lw=lw,solid_capstyle='round')
    if arrow:
        ax.annotate('',xy=pts[-1],xytext=pts[-2],arrowprops=dict(arrowstyle='-|>',color=c,lw=lw,mutation_scale=22))
    if label:
        x,y=lp; ax.text(x,y,label,ha='center',va='center',fontsize=15,color=c,bbox=dict(fc='white',ec='none',pad=2))
# left
box(40,110,360,150,'คอมพิวเตอร์ (PC)',['Robot Arm Controller','CODESYS IDE / WebVisu'],fc='#e8f1ff',ec=BLUE)
box(40,420,360,130,'Raspberry Pi 5',['CODESYS Control (Soft-PLC)'],fc='#e8f1ff',ec=BLUE)
box(40,700,360,110,'ตัวแปลง USB-to-TTL',['CH340'],)
# mega
mx,my,mw,mh=600,250,520,520
ax.add_patch(FancyBboxPatch((mx,my),mw,mh,boxstyle='round,pad=0,rounding_size=14',fc='#e8f1ff',ec=BLUE,lw=3))
ax.text(mx+mw/2,my+45,'Arduino Mega 2560',ha='center',fontsize=26,fontweight='bold',va='center')
ax.text(mx+mw/2,my+85,'Modbus RTU Server / สร้างพัลส์ขับมอเตอร์',ha='center',fontsize=15,va='center',color='#444')
L=[('USB-B',my+170),('D18 TX1 / D19 RX1',my+430)]
R=[('D2,4,6,8,10,12 → PUL',my+150),('D3,5,7,9,11,13 → DIR',my+210),('D30–D35 → ENA',my+270),('D20 SDA / D21 SCL',my+430)]
for t,y in L: ax.text(mx+18,y,t,fontsize=16,va='center',ha='left')
for t,y in R: ax.text(mx+mw-18,y,t,fontsize=16,va='center',ha='right')
# right
box(1330,170,330,170,'ไดรเวอร์ DM542T × 6',['PUL / DIR / ENA','A+ A− B+ B−'])
box(1720,170,170,170,'สเต็ปปิ้ง',['มอเตอร์ × 6','J1–J6'])
box(1330,20+0,330,90,'แหล่งจ่ายไฟ 24 V DC',[],fc='#ffe0e0',ec=RED,ts=20)
box(1330,560,330,170,'TCA9548A',['I2C Multiplexer (0x70)','SD0 SD1 SD6 SD5 SD7'])
box(1720,560,170,170,'AS5600 × 5',['เซนเซอร์มุม','J1–J5'],ts=19)
# wires
wire([(400,185),(500,185),(500,my+170),(mx,my+170)],BLUE,'USB Serial\n115200 bps',(500,330))
wire([(220,260),(220,420)],BLUE,'LAN',(220,340))
wire([(220,550),(220,700)],BLUE,'USB',(220,625))
wire([(400,755),(500,755),(500,my+430),(mx,my+430)],BLUE,'Modbus RTU\n115200 8N1',(500,620))
for y,yy in [(my+150,215),(my+210,255),(my+270,295)]:
    wire([(mx+mw,y),(1230,y),(1230,yy),(1330,yy)],GOLD,arrow=True)
ax.text(1450,400,'สัญญาณ PUL / DIR / ENA แกน J1–J6',fontsize=15,color=GOLD,ha='center')
wire([(1495,110),(1495,170)],RED,'+V / GND',(1580,140))
wire([(1660,255),(1720,255)],GOLD)
wire([(mx+mw,my+430),(1230,my+430),(1230,645),(1330,645)],GREEN,'I2C 400 kHz',(1230,600))
wire([(1660,645),(1720,645)],GREEN)
ax.text(W/2,870,'โหมด CODESYS: PC → LAN → Raspberry Pi 5 → USB → CH340 → Serial1 (Modbus RTU) → Arduino',ha='center',fontsize=17,color=BLUE)
ax.text(W/2,910,'โหมด Arduino: PC → USB Serial → Arduino      แขนกลรับคำสั่งได้ครั้งละหนึ่งโหมด',ha='center',fontsize=17,color=BLUE)
fig.savefig('/tmp/fc/circuit.png',dpi=150,facecolor='white')
