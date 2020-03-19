#ifndef FONTRENDERER_H_
#define FONTRENDERER_H_

#include <map>
#include <string>
#include <FTGL/ftgl.h>
#include <FTGL/FTGLPolygonFont.h>

namespace vlr {

class FontRenderer {
private:
	std::map<std::string, FTFont*> fontMap;
	FTFont* defaultFont;

public:
	FontRenderer();
	~FontRenderer();

	bool addFont(const std::string& fileName);
	bool addFont(const std::string& fontName, const unsigned char* mem, size_t memSize);
	void removeFont(const std::string& fontName);

	int numFonts() {return fontMap.size();}

	void drawString2D(const std::string& text, float x, float y, const std::string& fontName);
	void drawString2D(const std::string& text, float x, float y);
	void drawString2D(const std::string& text, float x, float y, float font_size);
//	void kogmo_graphics_draw_bitmap_string_centered_2D(float x, float y, void *font, char *string);
	void drawString3D(const std::string&, float x, float y, float z, const std::string& fontName);
	void drawString3D(const std::string&, float x, float y, float z);
//	int kogmo_graphics_bitmap_string_width(void *font, char *string);
//	double kogmo_graphics_stroke_text_width(void *font, float size, char *string);
//	void kogmo_graphics_draw_stroke_text_2D(float x, float y, void *font, float size, char *string);
//	void kogmo_graphics_draw_stroke_text_centered_2D(float x, float y, void *font, float size,char *string);
//	void kogmo_graphics_draw_stroke_string(void *font, char *string);
//	int kogmo_graphics_stroke_string_width(void *font, char *string);
};

} // namespace vlr

#endif // FONTRENDERER_H_
