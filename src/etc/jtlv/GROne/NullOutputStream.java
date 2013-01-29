import java.io.*;
/**Writes to nowhere*/
	public class NullOutputStream extends OutputStream {
	  @Override
	  public void write(int b) throws IOException {
	  }
	}
