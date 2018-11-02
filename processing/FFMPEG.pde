import java.nio.*;

ByteBuffer byteBuffer;
IntBuffer intBuffer;
OutputStream video;

OutputStream ffmpegOutputStream(String fname, float fps) {
  try {
    return new ProcessBuilder(
      "ffmpeg", // or "ffmepg"
      "-f", "rawvideo", // input format
      "-pix_fmt", "argb", // pixel format
      "-r", fps+"", // frame rate
      "-s", width+"x"+height, // input size (window size)
      "-i", "-", // input (stdin)
      "-y", // force overwrite
      "-qscale", "0", // highest quantization quality profile,
      fname // output file
      // inherit stderr to catch ffmpeg errors
    ).redirectError(ProcessBuilder.Redirect.INHERIT).start().getOutputStream(); 
  } catch (Exception e) {e.printStackTrace(); return null;}    
}

void ffmpegInit(String fname, float fps) {  
  video = ffmpegOutputStream(fname, fps);
  byteBuffer = ByteBuffer.allocate(width * height * 4);
  intBuffer = byteBuffer.asIntBuffer();  
}

void ffmpegCapture() {
  loadPixels();
  try {
    intBuffer.rewind();
    intBuffer.put(pixels);
    video.write(byteBuffer.array());
  } catch (IOException ioe) {}
}
