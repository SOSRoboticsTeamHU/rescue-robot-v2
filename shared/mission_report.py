"""
Mission Report Generator
Creates professional PDF reports for competition judges
"""

import sys
import time
from pathlib import Path
from typing import Dict, List, Optional
from datetime import datetime
from dataclasses import dataclass, asdict

try:
    from reportlab.lib import colors
    from reportlab.lib.pagesizes import letter, A4
    from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
    from reportlab.lib.units import inch
    from reportlab.platypus import (
        SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle,
        Image, PageBreak, KeepTogether
    )
    from reportlab.pdfbase import pdfmetrics
    from reportlab.pdfbase.ttfonts import TTFont
    REPORTLAB_AVAILABLE = True
except ImportError:
    REPORTLAB_AVAILABLE = False
    print("WARNING: reportlab not installed. PDF export will not work.")

# Import protocol types
sys.path.insert(0, str(Path(__file__).parent.parent))
from shared.protocol import (
    QRCodeDetection, HazmatDetection, LandoltCDetection,
    SLAMMapUpdate
)


@dataclass
class MissionReport:
    """Mission report data container"""
    mission_name: str
    team_name: str
    mission_start_time: float
    mission_end_time: float
    autonomous_time: float
    manual_time: float
    autonomous_ratio: float
    
    # Detections
    qr_codes: List[QRCodeDetection]
    hazmat_detections: List[HazmatDetection]
    landolt_detections: List[LandoltCDetection]
    slam_updates: List[SLAMMapUpdate]
    
    # Additional metadata
    notes: Optional[str] = None
    mission_status: str = "COMPLETED"  # COMPLETED, FAILED, INTERRUPTED
    
    def to_dict(self) -> Dict:
        """Convert to dictionary"""
        return {
            "mission_name": self.mission_name,
            "team_name": self.team_name,
            "mission_start_time": self.mission_start_time,
            "mission_end_time": self.mission_end_time,
            "autonomous_time": self.autonomous_time,
            "manual_time": self.manual_time,
            "autonomous_ratio": self.autonomous_ratio,
            "qr_codes": [qr.to_dict() for qr in self.qr_codes],
            "hazmat_detections": [h.to_dict() for h in self.hazmat_detections],
            "landolt_detections": [l.to_dict() for l in self.landolt_detections],
            "notes": self.notes,
            "mission_status": self.mission_status
        }
    
    @classmethod
    def from_dict(cls, data: Dict):
        """Create from dictionary"""
        qr_codes = [QRCodeDetection.from_dict(qr) for qr in data.get("qr_codes", [])]
        hazmat = [HazmatDetection.from_dict(h) for h in data.get("hazmat_detections", [])]
        landolt = [LandoltCDetection.from_dict(l) for l in data.get("landolt_detections", [])]
        
        return cls(
            mission_name=data["mission_name"],
            team_name=data["team_name"],
            mission_start_time=data["mission_start_time"],
            mission_end_time=data["mission_end_time"],
            autonomous_time=data["autonomous_time"],
            manual_time=data["manual_time"],
            autonomous_ratio=data["autonomous_ratio"],
            qr_codes=qr_codes,
            hazmat_detections=hazmat,
            landolt_detections=landolt,
            notes=data.get("notes"),
            mission_status=data.get("mission_status", "COMPLETED")
        )


class MissionReportGenerator:
    """Generate professional PDF mission reports"""
    
    def __init__(self, team_name: str = "TEAM LOGO", brand_color: str = "#00FF64"):
        self.team_name = team_name
        self.brand_color = brand_color
        
    def generate_pdf(self, report: MissionReport, output_path: str) -> bool:
        """Generate PDF report"""
        if not REPORTLAB_AVAILABLE:
            print("ERROR: reportlab not available. Cannot generate PDF.")
            return False
            
        try:
            doc = SimpleDocTemplate(output_path, pagesize=letter)
            story = []
            
            # Define styles
            styles = getSampleStyleSheet()
            title_style = ParagraphStyle(
                'CustomTitle',
                parent=styles['Heading1'],
                fontSize=24,
                textColor=colors.HexColor(self.brand_color),
                spaceAfter=30,
                alignment=1  # Center
            )
            
            heading_style = ParagraphStyle(
                'CustomHeading',
                parent=styles['Heading2'],
                fontSize=16,
                textColor=colors.HexColor(self.brand_color),
                spaceAfter=12
            )
            
            # Title
            story.append(Paragraph("MISSION REPORT", title_style))
            story.append(Spacer(1, 0.2*inch))
            
            # Mission Info Table
            mission_data = [
                ["Team:", report.team_name],
                ["Mission:", report.mission_name],
                ["Status:", report.mission_status],
                ["Start Time:", datetime.fromtimestamp(report.mission_start_time).strftime("%Y-%m-%d %H:%M:%S")],
                ["End Time:", datetime.fromtimestamp(report.mission_end_time).strftime("%Y-%m-%d %H:%M:%S")],
            ]
            
            mission_table = Table(mission_data, colWidths=[2*inch, 4*inch])
            mission_table.setStyle(TableStyle([
                ('BACKGROUND', (0, 0), (0, -1), colors.HexColor('#1E2430')),
                ('TEXTCOLOR', (0, 0), (-1, -1), colors.white),
                ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
                ('FONTNAME', (0, 0), (0, -1), 'Helvetica-Bold'),
                ('FONTNAME', (1, 0), (1, -1), 'Helvetica'),
                ('FONTSIZE', (0, 0), (-1, -1), 10),
                ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
                ('TOPPADDING', (0, 0), (-1, -1), 6),
                ('GRID', (0, 0), (-1, -1), 1, colors.HexColor('#3C465A')),
            ]))
            story.append(mission_table)
            story.append(Spacer(1, 0.3*inch))
            
            # Performance Metrics
            story.append(Paragraph("PERFORMANCE METRICS", heading_style))
            
            total_time = report.mission_end_time - report.mission_start_time
            autonomous_pct = report.autonomous_ratio * 100
            manual_pct = (1.0 - report.autonomous_ratio) * 100
            
            metrics_data = [
                ["Metric", "Value"],
                ["Total Mission Time", f"{total_time:.1f} seconds"],
                ["Autonomous Time", f"{report.autonomous_time:.1f} seconds ({autonomous_pct:.1f}%)"],
                ["Manual Time", f"{report.manual_time:.1f} seconds ({manual_pct:.1f}%)"],
                ["Autonomous Ratio", f"{autonomous_pct:.1f}%"],
            ]
            
            metrics_table = Table(metrics_data, colWidths=[3*inch, 3*inch])
            metrics_table.setStyle(TableStyle([
                ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#1E2430')),
                ('TEXTCOLOR', (0, 0), (-1, 0), colors.HexColor(self.brand_color)),
                ('TEXTCOLOR', (0, 1), (-1, -1), colors.white),
                ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
                ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
                ('FONTSIZE', (0, 0), (-1, -1), 10),
                ('BOTTOMPADDING', (0, 0), (-1, -1), 6),
                ('TOPPADDING', (0, 0), (-1, -1), 6),
                ('GRID', (0, 0), (-1, -1), 1, colors.HexColor('#3C465A')),
                ('BACKGROUND', (0, 1), (-1, -1), colors.HexColor('#141820')),
            ]))
            story.append(metrics_table)
            story.append(Spacer(1, 0.3*inch))
            
            # QR Code Detections
            if report.qr_codes:
                story.append(Paragraph("QR CODE DETECTIONS", heading_style))
                qr_data = [["#", "Content", "SLAM Coordinates", "Time", "Confidence"]]
                
                for i, qr in enumerate(report.qr_codes, 1):
                    coords_str = f"({qr.slam_coordinates[0]:.2f}, {qr.slam_coordinates[1]:.2f}, {qr.slam_coordinates[2]:.2f})"
                    time_str = datetime.fromtimestamp(qr.timestamp).strftime("%H:%M:%S")
                    qr_data.append([
                        str(i),
                        qr.content[:30] + "..." if len(qr.content) > 30 else qr.content,
                        coords_str,
                        time_str,
                        f"{qr.confidence:.2f}"
                    ])
                
                qr_table = Table(qr_data, colWidths=[0.5*inch, 2*inch, 1.5*inch, 1*inch, 1*inch])
                qr_table.setStyle(TableStyle([
                    ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#1E2430')),
                    ('TEXTCOLOR', (0, 0), (-1, 0), colors.HexColor(self.brand_color)),
                    ('TEXTCOLOR', (0, 1), (-1, -1), colors.white),
                    ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
                    ('FONTNAME', (0, 0), (-1, 0), 'Helvetica-Bold'),
                    ('FONTSIZE', (0, 0), (-1, -1), 9),
                    ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
                    ('TOPPADDING', (0, 0), (-1, -1), 4),
                    ('GRID', (0, 0), (-1, -1), 1, colors.HexColor('#3C465A')),
                    ('BACKGROUND', (0, 1), (-1, -1), colors.HexColor('#141820')),
                ]))
                story.append(qr_table)
                story.append(Spacer(1, 0.3*inch))
            else:
                story.append(Paragraph("QR CODE DETECTIONS: None", heading_style))
                story.append(Spacer(1, 0.1*inch))
            
            # Hazmat Detections
            if report.hazmat_detections:
                story.append(PageBreak())
                story.append(Paragraph("HAZMAT DETECTIONS", heading_style))
                
                for i, hazmat in enumerate(report.hazmat_detections, 1):
                    story.append(Paragraph(f"Detection #{i}: {hazmat.label_type}", styles['Heading3']))
                    
                    hazmat_data = [
                        ["Label Type:", hazmat.label_type],
                        ["Confidence:", f"{hazmat.confidence:.2f}"],
                        ["SLAM Coordinates:", f"({hazmat.slam_coordinates[0]:.2f}, {hazmat.slam_coordinates[1]:.2f}, {hazmat.slam_coordinates[2]:.2f})"],
                        ["Image Crop:", hazmat.image_crop_path],
                        ["Timestamp:", datetime.fromtimestamp(hazmat.timestamp).strftime("%Y-%m-%d %H:%M:%S")],
                    ]
                    
                    hazmat_table = Table(hazmat_data, colWidths=[2*inch, 4*inch])
                    hazmat_table.setStyle(TableStyle([
                        ('BACKGROUND', (0, 0), (0, -1), colors.HexColor('#1E2430')),
                        ('TEXTCOLOR', (0, 0), (-1, -1), colors.white),
                        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
                        ('FONTNAME', (0, 0), (0, -1), 'Helvetica-Bold'),
                        ('FONTSIZE', (0, 0), (-1, -1), 10),
                        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
                        ('TOPPADDING', (0, 0), (-1, -1), 4),
                        ('GRID', (0, 0), (-1, -1), 1, colors.HexColor('#3C465A')),
                    ]))
                    story.append(hazmat_table)
                    story.append(Spacer(1, 0.2*inch))
                    
            # Landolt-C Detections
            if report.landolt_detections:
                if not report.hazmat_detections:
                    story.append(PageBreak())
                else:
                    story.append(Spacer(1, 0.3*inch))
                    
                story.append(Paragraph("LANDOLT-C DETECTIONS", heading_style))
                
                for i, landolt in enumerate(report.landolt_detections, 1):
                    story.append(Paragraph(f"Detection #{i}: {landolt.orientation}", styles['Heading3']))
                    
                    landolt_data = [
                        ["Orientation:", landolt.orientation],
                        ["Confidence:", f"{landolt.confidence:.2f}"],
                        ["SLAM Coordinates:", f"({landolt.slam_coordinates[0]:.2f}, {landolt.slam_coordinates[1]:.2f}, {landolt.slam_coordinates[2]:.2f})"],
                        ["Image Crop:", landolt.image_crop_path],
                        ["Timestamp:", datetime.fromtimestamp(landolt.timestamp).strftime("%Y-%m-%d %H:%M:%S")],
                    ]
                    
                    landolt_table = Table(landolt_data, colWidths=[2*inch, 4*inch])
                    landolt_table.setStyle(TableStyle([
                        ('BACKGROUND', (0, 0), (0, -1), colors.HexColor('#1E2430')),
                        ('TEXTCOLOR', (0, 0), (-1, -1), colors.white),
                        ('ALIGN', (0, 0), (-1, -1), 'LEFT'),
                        ('FONTNAME', (0, 0), (0, -1), 'Helvetica-Bold'),
                        ('FONTSIZE', (0, 0), (-1, -1), 10),
                        ('BOTTOMPADDING', (0, 0), (-1, -1), 4),
                        ('TOPPADDING', (0, 0), (-1, -1), 4),
                        ('GRID', (0, 0), (-1, -1), 1, colors.HexColor('#3C465A')),
                    ]))
                    story.append(landolt_table)
                    story.append(Spacer(1, 0.2*inch))
            
            # Notes
            if report.notes:
                story.append(Spacer(1, 0.3*inch))
                story.append(Paragraph("ADDITIONAL NOTES", heading_style))
                story.append(Paragraph(report.notes, styles['Normal']))
            
            # Footer
            story.append(Spacer(1, 0.5*inch))
            story.append(Paragraph(
                f"Generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}",
                styles['Normal']
            ))
            
            # Build PDF
            doc.build(story)
            print(f"PDF report generated: {output_path}")
            return True
            
        except Exception as e:
            print(f"Error generating PDF: {e}")
            import traceback
            traceback.print_exc()
            return False


def create_mission_report_from_bridge(bridge, mission_name: str, team_name: str) -> MissionReport:
    """Create mission report from hardware bridge"""
    stats = bridge.get_mission_stats()
    
    report = MissionReport(
        mission_name=mission_name,
        team_name=team_name,
        mission_start_time=stats["mission_start_time"] or time.time(),
        mission_end_time=time.time(),
        autonomous_time=stats["autonomous_time"],
        manual_time=stats["manual_time"],
        autonomous_ratio=stats["autonomous_ratio"],
        qr_codes=bridge.qr_codes,
        hazmat_detections=bridge.hazmat_detections,
        landolt_detections=bridge.landolt_detections,
        slam_updates=bridge.slam_updates,
        mission_status="COMPLETED"
    )
    
    return report