
import re

def reverse_bits(n):
    return int('{:08b}'.format(n)[::-1], 2)

def process_file(filename):
    with open(filename, 'r') as f:
        content = f.read()

    # Regex to find array definitions: const unsigned char name[] PROGMEM = { ... };
    # We capture the name and the content
    pattern = re.compile(r'const\s+unsigned\s+char\s+(\w+)\s*\[\]\s*PROGMEM\s*=\s*\{([^}]+)\};', re.MULTILINE | re.DOTALL)
    
    def replacement(match):
        name = match.group(1)
        hex_data = match.group(2)
        
        # Parse hex values
        values = [int(x.strip(), 16) for x in re.findall(r'0x[0-9a-fA-F]+', hex_data)]
        
        # Reverse bits
        new_values = [reverse_bits(v) for v in values]
        
        # Reformat
        new_hex_data = ", ".join([f"0x{v:02x}" for v in new_values])
        
        # Wrap roughly
        wrapped_data = ""
        cnt = 0
        for val in new_hex_data.split(', '):
            wrapped_data += val + ", "
            cnt += 1
            if cnt % 12 == 0:
                wrapped_data += "\n"
        
        wrapped_data = wrapped_data.strip(", \n")
        
        return f"const unsigned char {name}[] PROGMEM = {{{wrapped_data}}};"

    new_content = pattern.sub(replacement, content)
    
    # Also uncomment and convert drawBitmap calls
    # Adafruit: display.drawBitmap(x, y, bitmap, w, h, color) 
    #   or display.drawBitmap(x, y, bitmap, w, h, color, bg)
    # U8g2: display.drawXBMP(x, y, w, h, bitmap)
    
    # Regex for commented out drawBitmap
    # //display.drawBitmap(x, y, bitmap, w, h, 1, 0);
    # Capture groups: 1=x, 2=y, 3=bitmap, 4=w, 5=h
    
    draw_pattern = re.compile(r'//\s*display\.drawBitmap\s*\(\s*([^,]+),\s*([^,]+),\s*([^,]+),\s*([^,]+),\s*([^,]+),\s*[^)]+\);')
    
    def draw_replacement(match):
        x = match.group(1).strip()
        y = match.group(2).strip()
        bitmap = match.group(3).strip()
        w = match.group(4).strip()
        h = match.group(5).strip()
        
        # U8g2 uses setDrawColor instead of passing it. We assume color 1.
        # drawXBMP(x, y, w, h, bitmap)
        return f"display.setDrawColor(1); display.drawXBMP({x}, {y}, {w}, {h}, {bitmap});"

    new_content = draw_pattern.sub(draw_replacement, new_content)

    with open(filename, 'w') as f:
        f.write(new_content)

process_file(r'c:\Users\ASUS\Documents\Arduino\AQI_Monitoring\AQI_Monitoring\AQI_Monitoring_oled\AQI_Monitoring_oled.ino')
