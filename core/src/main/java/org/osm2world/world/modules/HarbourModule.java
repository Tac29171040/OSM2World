package org.osm2world.world.modules;

import java.util.List;
import org.osm2world.map_data.data.*;
import org.osm2world.map_elevation.data.GroundState;
import org.osm2world.math.*;
import org.osm2world.scene.color.Color;
import org.osm2world.scene.material.Material;
import org.osm2world.world.data.NoOutlineWaySegmentWorldObject;
import org.osm2world.world.data.ProceduralWorldObject;
import org.osm2world.world.modules.common.ConfigurableWorldModule;

/** Metric harbour cross sections. The source way points with the seaward side on its right. */
public class HarbourModule extends ConfigurableWorldModule {
	static Material material(int r, int g, int b) {
		return new Material(Material.Interpolation.FLAT, new Color(r, g, b)).withDoubleSided(true);
	}
	static final Material WATER = material(60, 114, 128), BED = material(51, 63, 58);
	static final Material SAND = material(206, 184, 143), BANK = material(118, 112, 89);
	static final Material WALL = material(146, 149, 142), CAP = material(186, 183, 168);
	static final Material TIDE = material(92, 103, 91), STEEL = material(57, 62, 60);

	@Override public void applyTo(MapData data) {
		if (!config.getBoolean("awajiCoast", false)) return;
		for (var s : data.getMapWaySegments()) {
			if (s.getTags().containsKey("awaji:section")) s.addRepresentation(new HarbourSection(s));
		}
	}

	public static class HarbourSection extends NoOutlineWaySegmentWorldObject implements ProceduralWorldObject {
		HarbourSection(MapWaySegment segment) { super(segment); }
		@Override public GroundState getGroundState() { return GroundState.ON; }

		private VectorXZ offset(List<VectorXZ> line, int i, double width) {
			int last = line.size() - 1;
			int prev = i == 0 && line.getFirst().equals(line.getLast()) ? last - 1 : Math.max(0, i - 1);
			int next = i == last && line.getFirst().equals(line.getLast()) ? 1 : Math.min(last, i + 1);
			VectorXZ p = line.get(i);
			VectorXZ a = p.subtract(line.get(prev)), b = line.get(next).subtract(p);
			if (a.length() == 0) a = b;
			if (b.length() == 0) b = a;
			VectorXZ n = a.normalize().rightNormal(), m = b.normalize().rightNormal();
			return p.add(n.add(m).mult(width / Math.max(0.5, 1 + n.dot(m))));
		}

		private void panel(Target t, Material mat, List<VectorXZ> line, int i,
				double left, double right, double yLeft, double yRight, double rise) {
			var a = offset(line,i,left).xyz(yLeft);
			var b = offset(line,i,right).xyz(yRight);
			var c = offset(line,i+1,left).xyz(yLeft+rise);
			var d = offset(line,i+1,right).xyz(yRight+rise);
			t.drawTriangleStrip(mat, List.of(a,b,c,d), List.of());
		}

		@Override public void buildMeshesAndModels(Target t) {
			var tags = segment.getTags();
			String kind = tags.getValue("awaji:section");
			var line = segment.getWay().getPolylineXZ().vertices();
			int i = segment.getIndexInWay();
			if (kind.equals("subsoil")) {
				// Continuous substrate below the explicitly closed water bodies; also
				// backs subpixel raster cracks between independently exported land meshes.
				if (i == 0) {
					t.drawTriangleStrip(BED, List.of(line.get(0).xyz(-2.8), line.get(1).xyz(-2.8),
							line.get(3).xyz(-2.8), line.get(2).xyz(-2.8)), List.of());
				}
				return;
			}
			double distance = 0;
			for (int k=0; k<i; k++) distance += line.get(k).distanceTo(line.get(k+1));
			VectorXZ a = line.get(i), b = line.get(i+1);
			double len = a.distanceTo(b);
			if (len < .002) return;
			if (kind.equals("bank") || kind.equals("beach")) {
				double width = kind.equals("beach") ? 25 : 4.5;
				panel(t, kind.equals("beach") ? SAND : BANK, line, i, 0, width, 0, -2.5, 0);
				return;
			}
			if (kind.equals("vertical")) {
				panel(t,TIDE,line,i,0,0,0,-2.5,0);
				return;
			}
			if (kind.equals("slipway")) {
				double y = -distance/8;
				panel(t,WALL,line,i,-2.5,2.5,y,y,-len/8);
				return;
			}
			boolean pier = kind.equals("pier"), crest = kind.equals("breakwater");
			double left = kind.equals("quay") ? 0 : -2, right = kind.equals("quay") ? 1.5 : 2;
			double top = pier ? -.5 : crest ? 1 : 0;
			// A land-connected structure transitions from the shore over its first 8 m.
			boolean endAccess = tags.contains("awaji:access","end");
			boolean joined = endAccess || tags.contains("awaji:access","start");
			double approach = endAccess ? segment.getWay().getPolylineXZ().getLength()-distance : distance;
			String startTag = tags.getValue("awaji:ramp_start");
			double startGrade = startTag == null ? 0 : Double.parseDouble(startTag);
			approach -= startGrade;
			double y = joined ? top * Math.clamp(approach / 8,0,1) : top;
			double end = joined ? top * Math.clamp((approach + (endAccess ? -len : len)) / 8,0,1) : top;
			panel(t,CAP,line,i,left,right,y,y,end-y);
			for (double side : new double[]{left,right}) {
				double bottom = pier ? y-.28 : -2.5;
				panel(t,WALL,line,i,side,side,y,(pier ? bottom : y-0.65),end-y);
				if (!pier) {
					var u = offset(line,i,side);
					var v = offset(line,i+1,side);
					t.drawTriangleStrip(TIDE,List.of(u.xyz(y-.65),u.xyz(bottom),v.xyz(end-.65),v.xyz(bottom)),List.of());
				}
			}
			// End plates close the two exposed cross sections.
			for (int k : new int[]{i,i+1}) if (k==0 || k==line.size()-1) {
				double h = k==i ? y : end, floor = pier ? h-.28 : -2.5;
				VectorXZ u=offset(line,k,left), v=offset(line,k,right);
				t.drawTriangleStrip(WALL,List.of(u.xyz(floor),u.xyz(h),v.xyz(floor),v.xyz(h)),List.of());
			}
			if (!pier) {
				double side = crest ? left : right;
				double inset = crest ? side+.3 : side-.3;
				panel(t,CAP,line,i,inset,side,y+.15,y+.15,end-y);
				panel(t,WALL,line,i,inset,inset,y,y+.15,end-y);
				panel(t,WALL,line,i,side,side,y,y+.15,end-y);
				if (crest) {
					panel(t,WALL,line,i,right,right,y,y+1,end-y);
					panel(t,WALL,line,i,right-.35,right-.35,y,y+1,end-y);
					panel(t,CAP,line,i,right-.35,right,y+1,y+1,end-y);
				}
			}
			// Stations are measured continuously along the complete way.
			double spacing = pier ? 5 : 10;
			for (double station = spacing*(Math.ceil((distance-2)/spacing))+2; station < distance+len; station+=spacing) {
				double f=(station-distance)/len;
				VectorXZ mid=a.add(b.subtract(a).mult(f));
				VectorXZ n=b.subtract(a).normalize().rightNormal();
				double h=y+(end-y)*f;
				if (pier) {
					for (double side : new double[]{left+.35,right-.35})
						t.drawColumn(TIDE,8,mid.add(n.mult(side)).xyz(-2.5),h+2.23,.16,.16,false,false);
				} else {
					VectorXZ mooring=mid.add(n.mult(crest ? left+.5 : right-.5));
					t.drawColumn(STEEL,8,mooring.xyz(h),.32,.12,.115,false,true);
					t.drawColumn(STEEL,8,mooring.xyz(h+.32),.075,.18,.18,false,true);
				}
			}
		}
	}
}
