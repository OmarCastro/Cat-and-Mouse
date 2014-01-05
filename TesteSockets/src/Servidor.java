import java.net.InetSocketAddress;
import java.net.UnknownHostException;

import org.java_websocket.WebSocket;
import org.java_websocket.framing.Framedata;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;

/**
 * A simple WebSocketServer implementation. Keeps track of a "chatroom".
 */
public class Servidor extends WebSocketServer {

	public Servidor( int port ) throws UnknownHostException 
	{
		super( new InetSocketAddress( port ) );
	}

	public Servidor ( InetSocketAddress address ) 
	{
		super( address );
	}

	@Override
	public void onOpen( WebSocket conn, ClientHandshake handshake ) 
	{
		System.out.println( "Ligação estabelecida!" );
	}

	@Override
	public void onClose( WebSocket conn, int code, String reason, boolean remote ) 
	{
		System.out.println( conn + " terminou ligação!" );
	}

	@Override
	public void onMessage( WebSocket conn, String message ) 
	{
		String[] valores = message.split(" ");
		
		if(valores[0].compareTo("1") == 0)
		{
			//Valores provêm do controlador 1
			jClient.lmotorJogador1 = Double.parseDouble(valores[1]);
			jClient.rmotorJogador1 = Double.parseDouble(valores[2]);
		}
		else
		{
			//Valores provêm do controlador 2
			jClient.lmotorJogador2 = Double.parseDouble(valores[1]);
			jClient.rmotorJogador2 = Double.parseDouble(valores[2]);
		}
	}

	@Override
	public void onFragment( WebSocket conn, Framedata fragment ) 
	{
		System.out.println( "Fragmento: " + fragment );
	}
	
	@Override
	public void onError( WebSocket conn, Exception ex ) {
		ex.printStackTrace();
		if( conn != null ) {
			// some errors like port binding failed may not be assignable to a specific websocket
		}
	}

}