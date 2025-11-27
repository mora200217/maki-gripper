package com.meta.haptics_sdk_example;
import android.util.Log;
import org.json.JSONObject;
import org.json.JSONException;

public class JNIBridge {
    private static final String TAG = "JNIBridge";
    private static String serverUrl = "https://sherrell-presurgical-abe.ngrok-free.dev/metaquest/send";

    // Cargar la librería nativa
    static {
        try {
            System.loadLibrary("haptics_sdk_example"); // o el nombre de tu .so
            Log.d(TAG, "Librería nativa cargada exitosamente");
        } catch (UnsatisfiedLinkError e) {
            Log.e(TAG, "ERROR cargando librería nativa: " + e.getMessage());
        }
    }

    // Este método será llamado desde C++
    public static void onHapticEventFromNative(String eventType, String controller, float intensity) {
        Log.d(TAG, "=== onHapticEventFromNative INVOCADO ===");
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
        if (url == null || url.isEmpty()) {
            Log.e(TAG, "ERROR: URL inválida recibida");
            return;
        }
        serverUrl = url;
        Log.d(TAG, "Server URL set to: " + url);
    }

    public static void logCurrentState() {
        Log.d(TAG, "=== ESTADO ACTUAL JNI BRIDGE ===");
        Log.d(TAG, "Server URL: " + serverUrl);
        Log.d(TAG, "Clase cargada: Sí");
    }

    // Método para consultar comandos (llamado desde C++)
    public static void checkForCommandsFromNative() {
        new Thread(() -> {
            try {
                String response = HttpClient.getCommand(serverUrl.replace("/send", ""));
                Log.d(TAG, "Respuesta de comando: " + response);

                // Procesar la respuesta si hay comando disponible
                processCommandResponse(response);

            } catch (Exception e) {
                Log.e(TAG, "Error checking commands: " + e.getMessage());
            }
        }).start();
    }

    private static void processCommandResponse(String response) {
        try {
            JSONObject json = new JSONObject(response);

            if (json.getBoolean("command_available")) {
                JSONObject command = json.getJSONObject("command");

                // VALORES POR DEFECTO para campos opcionales
                float intensity = 5.0f; // valor por defecto
                float duration = 1000.0f; // valor por defecto
                String pattern = "continuous"; // valor por defecto

                // Manejar campos opcionales de forma segura
                if (command.has("intensity")) {
                    intensity = (float) command.getDouble("intensity");
                }
                if (command.has("duration")) {
                    duration = (float) command.getDouble("duration");
                }
                if (command.has("pattern")) {
                    pattern = command.getString("pattern");
                }

                Log.d(TAG, "Comando recibido - Intensidad: " + intensity +
                        ", Duración: " + duration + ", Patrón: " + pattern);

                Log.d(TAG, "Llamando función nativa onCommandReceived...");

                // Llamar de vuelta a C++ con el comando
                onCommandReceived(intensity, duration, pattern);

                Log.d(TAG, "Función nativa llamada exitosamente");
            }
        } catch (Exception e) {
            Log.e(TAG, "Error processing command: " + e.getMessage());
        }
    }

    // Método nativo para pasar el comando a C++
    private static native void onCommandReceived(float intensity, float duration, String pattern);
}
