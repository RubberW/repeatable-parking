"""Build the readable Chinese handoff PDF from the repository's Markdown source."""
from pathlib import Path
from xml.sax.saxutils import escape
import re
import json

from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT, TA_CENTER
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Flowable
from pypdf import PdfReader

ROOT = Path(__file__).resolve().parents[1]
SOURCE = ROOT/'docs/项目交接说明.md'
OUTPUT = ROOT/'output/pdf/重复停车项目交接说明.pdf'
pdfmetrics.registerFont(TTFont('Song', 'C:/Windows/Fonts/simsun.ttc', subfontIndex=0))
pdfmetrics.registerFont(TTFont('Hei', 'C:/Windows/Fonts/simhei.ttf'))
INK = colors.HexColor('#17212a')
BLUE = colors.HexColor('#244a62')
PALE = colors.HexColor('#eef3f6')
WIDTH = A4[0]-96
BODY = ParagraphStyle('Body',fontName='Song',fontSize=10.2,leading=16,spaceAfter=8,
                      wordWrap='CJK',textColor=INK)
TITLE = ParagraphStyle('Title',fontName='Hei',fontSize=23,leading=31,spaceAfter=13,textColor=colors.black)
H1 = ParagraphStyle('Heading',fontName='Hei',fontSize=17,leading=23,spaceAfter=13,textColor=colors.black)
H2 = ParagraphStyle('Subheading',fontName='Hei',fontSize=12,leading=18,spaceBefore=9,
                    spaceAfter=7,textColor=colors.black,keepWithNext=True)
LIST = ParagraphStyle('List',parent=BODY,leftIndent=14,firstLineIndent=-14,spaceAfter=5)
CODE = ParagraphStyle('Code',parent=BODY,fontName='Hei',fontSize=8.5,leading=13,
                      backColor=PALE,borderPadding=8,spaceBefore=3,spaceAfter=10)
CELL = ParagraphStyle('Cell',parent=BODY,fontSize=9.1,leading=14,spaceAfter=0)
HEAD = ParagraphStyle('HeaderCell',parent=CELL,fontName='Hei',textColor=colors.white)

def inline(text):
    text = escape(text)
    text = re.sub(r'`([^`]+)`', lambda m:'<font name="Hei">'+m.group(1)+'</font>', text)
    text = re.sub(r'\*\*([^*]+)\*\*',r'<font name="Hei">\1</font>',text)
    text = re.sub(r'\[([^\]]+)\]\(([^)]+)\)',r'\1',text)
    return text

class FlowDiagram(Flowable):
    def __init__(self):
        Flowable.__init__(self)
        self.width,self.height = WIDTH,171
    def draw(self):
        c=self.canv
        blocks=[(0,103,154,53,'观测与状态','反射率 编码器 时间'),
                (173,103,154,53,'探索主模型','选择可行方向与子目标'),
                (346,103,153,53,'条件运动执行器','驱动左右轮 实际转弯'),
                (173,12,154,52,'触黑框后 early_fit','定位 确认方向 停车'),
                (346,12,153,52,'条件触边小模型','接近平行 单轮保持内轮退')]
        for x,y,w,h,title,desc in blocks:
            c.setFillColor(PALE);c.setStrokeColor(colors.HexColor('#cbd5dc'))
            c.roundRect(x,y,w,h,6,fill=1,stroke=1)
            c.setFillColor(INK);c.setFont('Hei',10);c.drawCentredString(x+w/2,y+33,title)
            c.setFont('Song',9);c.drawCentredString(x+w/2,y+14,desc)
        def arrow(x1,y1,x2,y2):
            c.setStrokeColor(BLUE);c.setFillColor(BLUE);c.setLineWidth(1.3);c.line(x1,y1,x2,y2)
            p=c.beginPath();p.moveTo(x2,y2)
            if y1==y2:p.lineTo(x2-5,y2+3);p.lineTo(x2-5,y2-3)
            else:p.lineTo(x2-3,y2+5);p.lineTo(x2+3,y2+5)
            p.close();c.drawPath(p,fill=1,stroke=0)
        arrow(154,129,173,129);arrow(327,129,346,129)
        c.setStrokeColor(BLUE);c.line(422,103,422,82);c.line(422,82,250,82);arrow(250,82,250,64)
        c.setFont('Song',8.5);c.setFillColor(INK);c.drawString(265,87,'首次触黑框后切换')
        c.setDash(3,3);c.line(422,64,422,103);c.setDash()
        c.setFont('Song',8.5);c.drawString(358,72,'只接管触边执行')

def table(rows):
    cols=len(rows[0])
    if cols==3:
        widths=[WIDTH*.48,WIDTH*.21,WIDTH*.31]
    else:
        widths=[WIDTH*.47,WIDTH*.53]
    data=[[Paragraph(inline(cell), HEAD if n==0 else CELL) for cell in row] for n,row in enumerate(rows)]
    t=Table(data,colWidths=widths,repeatRows=1,hAlign='LEFT')
    t.setStyle(TableStyle([
        ('BACKGROUND',(0,0),(-1,0),BLUE),('ROWBACKGROUNDS',(0,1),(-1,-1),[colors.white,PALE]),
        ('GRID',(0,0),(-1,-1),.5,colors.HexColor('#d9dfe3')),
        ('VALIGN',(0,0),(-1,-1),'MIDDLE'),('LEFTPADDING',(0,0),(-1,-1),8),
        ('RIGHTPADDING',(0,0),(-1,-1),8),('TOPPADDING',(0,0),(-1,-1),6),
        ('BOTTOMPADDING',(0,0),(-1,-1),6)]))
    return t

def footer(canvas,doc):
    canvas.saveState()
    canvas.setFillColor(colors.HexColor('#687680'));canvas.setFont('Song',8.5)
    if doc.page>1:canvas.drawString(48,A4[1]-28,'重复停车项目交接说明')
    canvas.drawString(48,27,'记录日期 2026年10月14日    当前组合保持冻结')
    canvas.drawRightString(A4[0]-48,27,str(doc.page))
    canvas.restoreState()

def main():
    lines=SOURCE.read_text(encoding='utf-8').splitlines()
    story=[];i=0;first_section=True
    while i<len(lines):
        line=lines[i].strip()
        if not line:i+=1;continue
        if line.startswith('# '):story.append(Paragraph(inline(line[2:]),TITLE));i+=1;continue
        if line.startswith('## '):
            if not first_section:story.append(PageBreak())
            first_section=False
            story.append(Paragraph(inline(line[3:]),H1));i+=1;continue
        if line.startswith('### '):story.append(Paragraph(inline(line[4:]),H2));i+=1;continue
        if line.startswith('```'):
            kind=line[3:];body=[];i+=1
            while i<len(lines) and not lines[i].strip().startswith('```'):
                body.append(lines[i]);i+=1
            if kind=='mermaid':story += [FlowDiagram(),Spacer(1,7)]
            else:story.append(Paragraph('<br/>'.join(escape(t) for t in body),CODE))
            i+=1;continue
        if line.startswith('|'):
            rows=[]
            while i<len(lines) and lines[i].strip().startswith('|'):
                cells=[t.strip() for t in lines[i].strip().strip('|').split('|')]
                if not all(re.fullmatch(r':?-+:?',t) for t in cells):rows.append(cells)
                i+=1
            story += [table(rows),Spacer(1,10)];continue
        if re.match(r'^(\d+\. |[-] )',line):
            story.append(Paragraph(inline(line),LIST));i+=1;continue
        paragraph=[]
        while i<len(lines) and lines[i].strip() and not lines[i].startswith(('#','|','```')):
            paragraph.append(lines[i].strip());i+=1
        story.append(Paragraph(inline(' '.join(paragraph)),BODY))
    OUTPUT.parent.mkdir(parents=True,exist_ok=True)
    doc=SimpleDocTemplate(str(OUTPUT),pagesize=A4,leftMargin=48,rightMargin=48,
                         topMargin=49,bottomMargin=48,title='重复停车项目交接说明',author='SEMTM0043 项目记录')
    doc.build(story,onFirstPage=footer,onLaterPages=footer)
    reader=PdfReader(OUTPUT)
    all_text='\n'.join(page.extract_text() or '' for page in reader.pages)
    for expected in ['heading_seed4','early_fit','190.67','19.91','104.41','4597']:
        assert expected in all_text,expected
    out=ROOT/'tmp/pdfs/handoff_20261014'
    out.mkdir(parents=True,exist_ok=True)
    (out/'text.txt').write_text(all_text,encoding='utf-8')
    print(json.dumps(dict(pdf=str(OUTPUT),pages=len(reader.pages),bytes=OUTPUT.stat().st_size),ensure_ascii=False))

if __name__=='__main__':main()
