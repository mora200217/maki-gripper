package com.meta.haptics_sdk_example;
import android.util.Log;
public class JNIBridge {
    private static final String TAG = "JNIBridge";
    private static String serverUrl = "https://sherrell-presurgical-abe.ngrok-free.dev/metaquest/send";

    // Este método será llamado desde C++
    public static void onHapticEventFromNative(String eventType, String controller, float intensity) {
        Log.d(TAG, "Received from native: " + eventType + " - " + controller + " - " + intensity);

        // Usar HttpClient para enviar al servidor
        HttpClient.sendHapticEvent(serverUrl, eventType, controller, intensity,
                new HttpClient.HttpResponseCallback() {
                    @Override
                    public void onSuccess(String response) {
                        Log.d(TAG, "Successfully sent to server: " + response);
                    }

                    @Override
                    public void onError(String error) {
                        Log.e(TAG, "Failed to send to server: " + error);
                    }
                });
    }

    // Método para configurar la URL del servidor
    public static void setServerUrl(String url) {
        serverUrl = url;
        Log.d(TAG, "Server URL set to: " + url);
    }
}
