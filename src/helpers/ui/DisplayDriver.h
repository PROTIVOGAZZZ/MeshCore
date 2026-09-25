#pragma once

#include <stdint.h>
#include <string.h>

using ColorVal = uint16_t;

class UIColor {
public:
  // color definitions (by element _type_)
  static ColorVal window_bkg, title_bkg, title_txt, primary_txt, secondary_txt, warning_txt, popup_bkg, popup_txt, corp_blue;
};

class DisplayDriver {
  int _w, _h;
protected:
  DisplayDriver(int w, int h) { _w = w; _h = h; }
#ifdef CYRILLIC_SUPPORT
  uint8_t _font_size = 1;     // current text size; used for baseline offset and word-wrap metrics
  int _cursor_y_raw = 0;      // logical y before GFXfont baseline shift; tracks line position in printWordWrap
#endif
public:
  //enum Color { DARK=0, LIGHT, RED, GREEN, BLUE, YELLOW, ORANGE }; // on b/w screen, colors will be !=0 synonym of light

  int width() const { return _w; }
  int height() const { return _h; }

  virtual bool isOn() = 0;
  virtual bool isEink() { return false; } // default to non-eink, override in eink drivers
  virtual void turnOn() = 0;
  virtual void turnOff() = 0;
  virtual void clear() = 0;
  virtual void startFrame(ColorVal bkg = UIColor::window_bkg) = 0;
  virtual void setTextSize(int sz) = 0;
  virtual void setColor(ColorVal c) = 0;
  virtual void setCursor(int x, int y) = 0;
  virtual void print(const char* str) = 0;
#ifdef CYRILLIC_SUPPORT
  // glcdfont6x8 is monospace (6x8 px per glyph, scaled by _font_size); str must already be CP1251
  virtual void printWordWrap(const char* str, int max_width) {
    const int char_w = 6 * _font_size;
    const int line_h = 8 * _font_size;
    const int max_chars = max_width / char_w;
    const int x0 = 0;
    int len = (int)strlen(str);
    int pos = 0;
    char line_buf[64];

    while (pos < len && _cursor_y_raw + line_h <= height()) {
      if (len - pos <= max_chars) {
        print(str + pos);
        break;
      }
      int break_at = pos + max_chars;   // prefer breaking on a space
      for (int i = pos + max_chars; i > pos; i--) {
        if (str[i] == ' ') { break_at = i; break; }
      }
      int seg_len = break_at - pos;
      if (seg_len > (int)sizeof(line_buf) - 1) seg_len = (int)sizeof(line_buf) - 1;
      memcpy(line_buf, str + pos, seg_len);
      line_buf[seg_len] = 0;
      print(line_buf);

      pos = break_at + (str[break_at] == ' ' ? 1 : 0);
      int next_y = _cursor_y_raw + line_h;
      if (next_y + line_h > height()) break;
      setCursor(x0, next_y);
    }
  }
#else
  virtual void printWordWrap(const char* str, int max_width) { print(str); }   // fallback to basic print() if no override
#endif
  virtual void fillRect(int x, int y, int w, int h) = 0;
  virtual void drawRect(int x, int y, int w, int h) = 0;
  virtual void drawXbm(int x, int y, const uint8_t* bits, int w, int h) = 0;
  virtual uint16_t getTextWidth(const char* str) = 0;
  virtual void drawTextCentered(int mid_x, int y, const char* str) {   // helper method (override to optimise)
    int w = getTextWidth(str);
    setCursor(mid_x - w/2, y);
    print(str);
  }
  virtual void drawTextRightAlign(int x_anch, int y, const char* str) {
    int w = getTextWidth(str);
    setCursor(x_anch - w, y);
    print(str);
  }
  virtual void drawTextLeftAlign(int x_anch, int y, const char* str) {
    setCursor(x_anch, y);
    print(str);
  }
  
#ifdef CYRILLIC_SUPPORT
  // Map a Unicode code point to CP1251 (the encoding of glcdfont6x8); 0 = no glyph
  static uint8_t unicodeToCP1251(uint32_t u) {
    if (u >= 0x410 && u <= 0x44F) return (uint8_t)(u - 0x410 + 0xC0);   // А-я
    if (u >= 0xA0 && u <= 0xBF) {                                         // Latin-1 symbols shared with CP1251
      switch (u) {
        case 0xA0: case 0xA4: case 0xA6: case 0xA7: case 0xA9: case 0xAB: case 0xAC: case 0xAD: case 0xAE:
        case 0xB0: case 0xB1: case 0xB5: case 0xB6: case 0xB7: case 0xBB: return (uint8_t)u;
      }
      return 0;
    }
    switch (u) {
      case 0x401: return 0xA8; case 0x451: return 0xB8;   // Ё ё
      case 0x404: return 0xAA; case 0x454: return 0xBA;   // Є є
      case 0x406: return 0xB2; case 0x456: return 0xB3;   // І і
      case 0x407: return 0xAF; case 0x457: return 0xBF;   // Ї ї
      case 0x40E: return 0xA1; case 0x45E: return 0xA2;   // Ў ў
      case 0x490: return 0xA5; case 0x491: return 0xB4;   // Ґ ґ
      case 0x2013: return 0x96; case 0x2014: return 0x97; // – —
      case 0x2018: return 0x91; case 0x2019: return 0x92; // ‘ ’
      case 0x201C: return 0x93; case 0x201D: return 0x94; // “ ”
      case 0x201E: return 0x84; case 0x2026: return 0x85; // „ …
      case 0x2022: return 0x95; case 0x20AC: return 0x88; // • €
      case 0x2116: return 0xB9; case 0x2122: return 0x99; // № ™
    }
    return 0;
  }

  // Convert UTF-8 to CP1251 for glcdfont6x8. Idempotent: bytes that do not form a valid
  // UTF-8 sequence are treated as already-converted CP1251 and passed through, so the
  // UI may translate a string and the driver's print() translate it again safely.
  // Characters without a glyph (emoji etc.) become '?'.
  virtual void translateUTF8ToBlocks(char* dest, const char* src, size_t dest_size) {
    size_t j = 0;
    const uint8_t* s = (const uint8_t*)src;
    for (size_t i = 0; s[i] != 0 && j < dest_size - 1; ) {
      uint8_t c = s[i];
      int n = (c >= 0xC2 && c <= 0xDF) ? 1 : (c >= 0xE0 && c <= 0xEF) ? 2 : (c >= 0xF0 && c <= 0xF4) ? 3 : 0;
      bool valid = n > 0;
      for (int k = 1; valid && k <= n; k++) valid = (s[i + k] & 0xC0) == 0x80;
      if (valid) {
        uint32_t u = c & (0x3F >> n);
        for (int k = 1; k <= n; k++) u = (u << 6) | (s[i + k] & 0x3F);
        uint8_t cp = unicodeToCP1251(u);
        if (u == 0xFE0F || u == 0x200D) { i += n + 1; continue; }   // invisible emoji modifiers
        dest[j++] = cp ? (char)cp : '?';
        i += n + 1;
      } else {
        if (c >= 32 && c != 127) dest[j++] = (char)c;   // ASCII or already CP1251
        i++;
      }
    }
    dest[j] = 0;
  }
#else
  // convert UTF-8 characters to displayable block characters for compatibility
  virtual void translateUTF8ToBlocks(char* dest, const char* src, size_t dest_size) {
    size_t j = 0;
    for (size_t i = 0; src[i] != 0 && j < dest_size - 1; i++) {
      unsigned char c = (unsigned char)src[i];
      if (c >= 32 && c <= 126) {
        dest[j++] = c;  // ASCII printable
      } else if (c >= 0x80) {
        dest[j++] = '\xDB';  // CP437 full block █
        while (src[i+1] && (src[i+1] & 0xC0) == 0x80) 
          i++;  // skip UTF-8 continuation bytes
      }
    }
    dest[j] = 0;
  }
#endif
  
  // draw text with ellipsis if it exceeds max_width
  virtual void drawTextEllipsized(int x, int y, int max_width, const char* str) {
    char temp_str[256];  // reasonable buffer size
    size_t len = strlen(str);
    if (len >= sizeof(temp_str)) len = sizeof(temp_str) - 1;
    memcpy(temp_str, str, len);
    temp_str[len] = 0;
    
    if (getTextWidth(temp_str) <= max_width) {
      setCursor(x, y);
      print(temp_str);
      return;
    }
    
    // for variable-width fonts (GxEPD), add space after ellipsis
    // for fixed-width fonts (OLED), keep tight spacing to save precious characters
    const char* ellipsis;
    // use a simple heuristic: if 'i' and 'l' have different widths, it's variable-width
    int i_width = getTextWidth("i");
    int l_width = getTextWidth("l");
    if (i_width != l_width) {
      ellipsis = "... ";  // variable-width fonts: add space
    } else {
      ellipsis = "...";   // fixed-width fonts: no space
    }
    
    int ellipsis_width = getTextWidth(ellipsis);
    int str_len = strlen(temp_str);
    
    while (str_len > 0 && getTextWidth(temp_str) > max_width - ellipsis_width) {
      temp_str[--str_len] = 0;
    }
    strcat(temp_str, ellipsis);
    
    setCursor(x, y);
    print(temp_str);
  }
  
  virtual void endFrame() = 0;
};
