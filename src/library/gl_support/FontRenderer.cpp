#include <GL/gl.h>

#include "aw_arial.h"
#include "FontRenderer.h"

namespace vlr {

FontRenderer::FontRenderer() : defaultFont(0) {
  if (!addFont("arial", arial_resource_data, arial_resource_size)) {
    throw("Cannot add default font resource.");
  }
  defaultFont = (*fontMap.begin()).second;
//  defaultFont->UseDisplayList(false);
}

FontRenderer::~FontRenderer() {
  std::map<std::string, FTFont*>::const_iterator fit, fit_end;

  for (fit = fontMap.begin(), fit_end = fontMap.end(); fit != fit_end; ++fit) {
    delete (*fit).second;
  }
}

bool FontRenderer::addFont(const std::string& fileName) {
  FTFont* font = NULL;

  try {
    font = new FTGLPolygonFont(fileName.c_str());
  }
  catch (...) {
    return false;
  }

  font->FaceSize(10);

  //TODO: Extract fontName from file / fileName
  std::pair<std::map<std::string, FTFont*>::iterator, bool> pair = fontMap.insert(
      std::make_pair(fileName.c_str(), font));

  if (pair.second) {
    return true;
  }

  delete font;

  return false;
}

bool FontRenderer::addFont(const std::string& fontName, const unsigned char* mem, size_t memSize) {
  FTFont* font = 0;

  if (!mem) {
    return false;
  }

  try {
    font = new FTGLPolygonFont(mem, memSize);
  }
  catch (...) {
    return false;
  }

  font->FaceSize(10);

  std::pair<std::map<std::string, FTFont*>::iterator, bool> pair = fontMap.insert(std::make_pair(fontName, font));

  if (pair.second) {
    return true;
  }

  delete font;

  return false;
}

void FontRenderer::removeFont(const std::string& fontName) {
  std::map<std::string, FTFont*>::iterator fit = fontMap.find(fontName);

  if (fit == fontMap.end()) {
    return;
  }

  FTFont* font = (*fit).second;

  fontMap.erase(fit);

  delete font;
}

void FontRenderer::drawString2D(const std::string& text, float x, float y, const std::string& fontName) {
  std::map<std::string, FTFont*>::iterator fit = fontMap.find(fontName);

  if (fit == fontMap.end()) {
    return;
  }

  FTFont* font = (*fit).second;

  glPushMatrix();
  glTranslatef(x, y, 0.);

  glScalef(0.1, 0.1, 1); // compatibility scale...

  font->Render(text.c_str());
  glPopMatrix();
}

void FontRenderer::drawString2D(const std::string& text, float x, float y) {
  return; // TODO: Fix performance issue
  glPushMatrix();
  glTranslatef(x, y, 0.);

  glScalef(0.1, 0.1, 1); // compatibility scale...

  defaultFont->Render(text.c_str());
  glPopMatrix();
}

void FontRenderer::drawString2D(const std::string& text, float x, float y, float font_size) {
//  return; // TODO: Fix performance issue
  glPushMatrix();
  glTranslatef(x, y, 0.);

  glScalef(0.1*font_size, 0.1*font_size, 1); // compatibility scale...

  defaultFont->Render(text.c_str());
  glPopMatrix();
}

void FontRenderer::drawString3D(const std::string& text, float x, float y, float z, const std::string& fontName) {
  std::map<std::string, FTFont*>::iterator fit = fontMap.find(fontName);

  if (fit == fontMap.end()) {
    return;
  }

  FTFont* font = (*fit).second;

  glPushMatrix();
  glTranslatef(x, y, z);

  glScalef(0.1, 0.1, 1); // compatibility scale...

  font->Render(text.c_str());
  glPopMatrix();
}

void FontRenderer::drawString3D(const std::string& text, float x, float y, float z) {
  glPushMatrix();
  glTranslatef(x, y, z);

  glScalef(0.1, 0.1, 1); // compatibility scale...

  defaultFont->Render(text.c_str());
  glPopMatrix();
}

} // namespace vlr
