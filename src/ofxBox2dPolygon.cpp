/*
 *  ofxBox2dPolygon.cpp
 *  ofxBox2d
 *
 *  Created by Todd Vanderlin on 1/14/11.
 *  Copyright 2011 Interactive Design. All rights reserved.
 *  Modifed by Daniel Rosser <danoli3@gmail.com> on 7/7/13 for Box2D 2.* +
 *
 */

#include "ofxBox2dPolygon.h"
#include "ofxBox2dPolygonUtils.h"

//----------------------------------------
ofxBox2dPolygon::ofxBox2dPolygon() { 
	bIsTriangulated = false;
	bIsSimplified   = false;
	bSetAsEdge	    = false;
	bSetAsChain     = false;
}

//----------------------------------------
ofxBox2dPolygon::~ofxBox2dPolygon() { 
}

//----------------------------------------
void ofxBox2dPolygon::destroy() {
	ofxBox2dBaseShape::destroy();
	clear();
}

//----------------------------------------
void ofxBox2dPolygon::setup(b2World * b2dworld) {
	
	
	
}

//----------------------------------------
float ofxBox2dPolygon::calculateArea() {
	int i, j, n = (int)size();
	float polyArea = 0;
	for (i = 0; i < n; i++) {
		j = (i + 1) % n;
		polyArea += getVertices()[i].x * getVertices()[j].y;
		polyArea -= getVertices()[j].x * getVertices()[i].y;
	}
	polyArea /= 2.0;
	area = polyArea;
	return area;
}

//----------------------------------------
void ofxBox2dPolygon::calculateCentroid() {
	float cx = 0, cy = 0;
	int i, j, n = (int)size();
	
	float factor = 0;
	for (i = 0; i < n; i++) {
		j = (i + 1) % n;
		factor = (getVertices()[i].x * getVertices()[j].y - getVertices()[j].x * getVertices()[i].y);
		cx += (getVertices()[i].x + getVertices()[j].x) * factor;
		cy += (getVertices()[i].y + getVertices()[j].y) * factor;
	}
	area *= 6.0f;
	factor = 1 / area;
	cx *= factor;
	cy *= factor;
	center.set(cx, cy);	
}

//----------------------------------------
ofVec2f ofxBox2dPolygon::getCenter() {	
	if(!body) return ofVec2f(0,0);
	const b2Transform& xf = body->GetTransform();
	b2Vec2 b2Center = b2Mul(xf, b2Vec2(center.x/OFX_BOX2D_SCALE, center.y/OFX_BOX2D_SCALE));
	return ofVec2f(b2Center.x*OFX_BOX2D_SCALE, b2Center.y*OFX_BOX2D_SCALE);
};

//----------------------------------------
void ofxBox2dPolygon::updateShape() {
	
	calculateArea();
	calculateCentroid();
	bounds = getBoundingBox();
}

//----------------------------------------
void ofxBox2dPolygon::addTriangle(const ofVec2f &a, const ofVec2f &b, const ofVec2f &c) {
	addVertex(a); addVertex(b);	addVertex(c);
	
	// dont forget to close it
	addVertex(a);
	close();
}

/*
 These were in ofPolyline and now are gone?
*/
//----------------------------------------
void ofxBox2dPolygon::addVertexes(vector <ofVec2f> &pts) {
	for (int i=0; i<pts.size(); i++) {
		addVertex(pts[i].x, pts[i].y);
	}
}

//----------------------------------------
void ofxBox2dPolygon::addVertexes(ofPolyline &polyline) {
	for (int i=0; i<polyline.size(); i++) {
		addVertex(polyline[i].x, polyline[i].y);
	}
}


//----------------------------------------
void ofxBox2dPolygon::simplify(float tolerance) {
	ofPolyline::simplify(tolerance);	
	bIsSimplified = true;
}

//----------------------------------------
void ofxBox2dPolygon::triangulate(float resampleAmt, int nPointsInside) {
	
	triangles.clear();
	
	if(size() > 0) {
		
		// copy over the points into a polyline
		ofPolyline polyOutline;
		ofPolyline newPoly;
		
		// make sure to close the polyline and then
		// simplify and resample by spacing...
		setClosed(true);
		if(!bIsSimplified) simplify();
		newPoly = getResampledBySpacing(resampleAmt);
		
		// save the outline...
		polyOutline = newPoly;
		
		// add some random points inside then
		// triangulate the polyline...
		if(nPointsInside!=-1) addRandomPointsInside(newPoly, nPointsInside);
		triangles = triangulatePolygonWithOutline(newPoly, polyOutline);
			
		clear();
		
		// now add back into polyshape
		for (int i=0; i<newPoly.size(); i++) {
			addVertex(newPoly[i]);
		}
	}
	
	bIsTriangulated = true;
}


//----------------------------------------
void ofxBox2dPolygon::create(b2World * b2dworld) {

	if(size() < 3) {
		printf("need at least 3 points\n");
		return;	
	}
	
	if(size() >= b2_maxPolygonVertices) {
		ofLogNotice("polygon size '" + ofToString(size()) + "' is larger than box2d maximum... making into a chain");
		bSetAsChain = true;
	}
	
	if (body != NULL) {
		b2dworld->DestroyBody(body);
		body = NULL;
	}
	
	// create the body from the world (1)
	b2BodyDef		bd;
	bd.type			= density <= 0.0 ? b2_staticBody : b2_dynamicBody;
	body			= b2dworld->CreateBody(&bd);

	if(bIsTriangulated) {
		bSetAsChain = false;
		b2PolygonShape	shape;
		b2FixtureDef	fixture;
		b2Vec2			verts[3];
		
		ofVec2f a, b, c;
		for (int i=0; i<triangles.size(); i++) {
			
			a = triangles[i].a;
			b = triangles[i].b;
			c = triangles[i].c;

			verts[0].Set(a.x/OFX_BOX2D_SCALE, a.y/OFX_BOX2D_SCALE);
			verts[1].Set(b.x/OFX_BOX2D_SCALE, b.y/OFX_BOX2D_SCALE);
			verts[2].Set(c.x/OFX_BOX2D_SCALE, c.y/OFX_BOX2D_SCALE);
			
			shape.Set(verts, 3);
			
			fixture.density		= density;
			fixture.restitution = bounce;
			fixture.friction	= friction;
			fixture.shape		= &shape;
			body->CreateFixture(&fixture);
		}
	
	}
	else {
		if(bSetAsEdge) {
			for (int i=1; i<size(); i++) {
				b2EdgeShape shape;
				b2Vec2 a = screenPtToWorldPt(getVertices()[i-1]);
				b2Vec2 b = screenPtToWorldPt(getVertices()[i]);
				shape.Set(a, b);
				fixture.shape		= &shape;
				fixture.density		= density;
				fixture.restitution = bounce;
				fixture.friction	= friction;	
				
				body->CreateFixture(&fixture);
			}	
		} else if(bSetAsChain) {
			
			vector<b2Vec2> points;
			for (int i=0; i<size(); i++) {
				points.push_back(b2Vec2(getVertices()[i].x / OFX_BOX2D_SCALE, getVertices()[i].y / OFX_BOX2D_SCALE));
			}
			b2ChainShape	chain;
			chain.CreateChain(&points[0], points.size());
			fixture.shape		= &chain;
			fixture.density		= density;
			fixture.restitution = bounce;
			fixture.friction	= friction;
			
			body->CreateFixture(&fixture);
			
		}
		else {
			
			b2PolygonShape	shape;
			shape.m_count = int(size());
			for (int i=0; i<size(); i++) {
				shape.m_vertices[i].Set(getVertices()[i].x / OFX_BOX2D_SCALE, getVertices()[i].y / OFX_BOX2D_SCALE);
			}
			shape.m_count = int(size());
			shape.Set(shape.m_vertices, shape.m_count);
			
			fixture.shape		= &shape;
			fixture.density		= density;
			fixture.restitution = bounce;
			fixture.friction	= friction;	
			
			body->CreateFixture(&fixture);
		}		
			
	}
	
	// update the area and centroid
	updateShape();
}

//------------------------------------------------
void ofxBox2dPolygon::addAttractionPoint (ofVec2f pt, float amt) {
    // we apply forces at each vertex. 
    if(body != NULL) {
        const b2Transform& xf = body->GetTransform();
		
        for (b2Fixture* f = body->GetFixtureList(); f; f = f->GetNext()) {
			if(bSetAsEdge) {
				// @TODO
				
			} else if(bSetAsChain) {
				b2ChainShape* chain = (b2ChainShape*)f->GetShape();
				
				if(chain) {
					b2Vec2 P(pt.x/OFX_BOX2D_SCALE, pt.y/OFX_BOX2D_SCALE);
					
					for(int i=0; i<chain->m_count; i++) {
						b2Vec2 qt = b2Mul(xf, chain->m_vertices[i]);
						b2Vec2 D = P - qt;
						b2Vec2 F = amt * D;
						body->ApplyForce(F, P, active);
					}
				}
				
			} else {
				b2PolygonShape* poly = (b2PolygonShape*)f->GetShape();
				
				if(poly) {
					b2Vec2 P(pt.x/OFX_BOX2D_SCALE, pt.y/OFX_BOX2D_SCALE);
					
					for(int i=0; i<poly->GetVertexCount(); i++) {
						b2Vec2 qt = b2Mul(xf, poly->GetVertex(i));
						b2Vec2 D = P - qt; 
						b2Vec2 F = amt * D;
						body->ApplyForce(F, P, active);
					}                    
				}
			}
        }
    }
}


//----------------------------------------
void ofxBox2dPolygon::addAttractionPoint (float x, float y, float amt) {
    addAttractionPoint(ofVec2f(x, y), amt);
}

//----------------------------------------
void ofxBox2dPolygon::addRepulsionForce(float x, float y, float amt) {
	addRepulsionForce(ofVec2f(x, y), amt);
}
void ofxBox2dPolygon::addRepulsionForce(ofVec2f pt, float amt) {
	// we apply forces at each vertex. 
    if(body != NULL) {
        const b2Transform& xf = body->GetTransform();
		
        for (b2Fixture* f = body->GetFixtureList(); f; f = f->GetNext()) {
			if(bSetAsEdge) {
				b2EdgeShape* edge = (b2EdgeShape*)f->GetShape();
				
				if(edge) {
					b2Vec2 P(pt.x/OFX_BOX2D_SCALE, pt.y/OFX_BOX2D_SCALE);
					
//					for(int i=0; i<edge->m_ i++) {
//						b2Vec2 qt = b2Mul(xf, edge->GetVertex(i));
//						b2Vec2 D = P - qt;
//						b2Vec2 F = amt * D;
//						body->ApplyForce(-F, P, active);
//					}
				}
			}
			else if(bSetAsChain) {
				b2ChainShape* chain = (b2ChainShape*)f->GetShape();
				
				if(chain) {
					b2Vec2 P(pt.x/OFX_BOX2D_SCALE, pt.y/OFX_BOX2D_SCALE);
					
					for(int i=0; i<chain->m_count; i++) {
						b2Vec2 qt = b2Mul(xf, chain->m_vertices[i]);
						b2Vec2 D = P - qt;
						b2Vec2 F = amt * D;
						body->ApplyForce(-F, P, active);
					}
				}
			}
			else {
				
				b2PolygonShape* poly = (b2PolygonShape*)f->GetShape();
				
				if(poly) {
					b2Vec2 P(pt.x/OFX_BOX2D_SCALE, pt.y/OFX_BOX2D_SCALE);
					
					for(int i=0; i<poly->GetVertexCount(); i++) {
						b2Vec2 qt = b2Mul(xf, poly->GetVertex(i));
						b2Vec2 D = P - qt;
						b2Vec2 F = amt * D;
						body->ApplyForce(-F, P, active);
					}
				}
			}
			
			}
        }
}

//----------------------------------------
void ofxBox2dPolygon::draw() {
	
	//ofNoFill();
	//ofRect(bounds);
	//ofVec2f centroid = getCenter();
	//ofCircle(centroid.x, centroid.y, 2);
	
	if(body == NULL) {
		return;	
	}
	
	const b2Transform& xf = body->GetTransform();
	
	for (b2Fixture * f = body->GetFixtureList(); f; f = f->GetNext()) {
		if(bSetAsEdge) {
			//@TODO
		}
		else if(bSetAsChain) {
			b2ChainShape* chain = (b2ChainShape*)f->GetShape();
			
			if(chain) {
				drawShape.clear();
					for(int i=0; i<chain->m_count; i++) {
						b2Vec2 pt = b2Mul(xf, chain->m_vertices[i]);
						drawShape.addVertex(pt.x*OFX_BOX2D_SCALE, pt.y*OFX_BOX2D_SCALE);
					}
					if(isClosed()) drawShape.close();
				drawShape.draw();
			}
			
		} else {
				
			b2PolygonShape * poly = (b2PolygonShape*)f->GetShape();
			
			if(poly) {
				drawShape.clear();
				for(int i=0; i<poly->GetVertexCount(); i++) {
					b2Vec2 pt = b2Mul(xf, poly->GetVertex(i));
					drawShape.addVertex(pt.x*OFX_BOX2D_SCALE, pt.y*OFX_BOX2D_SCALE);
				}
				if(isClosed()) drawShape.close();
				drawShape.draw();
			
			}
		}
	}
	
	
	
}












