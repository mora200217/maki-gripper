package com.meta.haptics_sdk_example;

import android.util.Log;
import org.json.JSONObject;
import java.io.BufferedReader;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.HttpURLConnection;
import java.net.URL;
import javax.net.ssl.HttpsURLConnection;

public class HttpClient {
    private static final String TAG = "HttpClient";
    private static final int TIMEOUT_MS = 5000;

    public interface HttpResponseCallback {
        void onSuccess(String response);
        void onError(String error);
    }

    public static void sendHapticEvent(String serverUrl, String eventType,
                                       String controller, float intensity,
                                       HttpResponseCallback callback) {
        new Thread(() -> {
            HttpURLConnection connection = null;
            try {
                URL url = new URL(serverUrl);
                connection = (HttpURLConnection) url.openConnection();

                if (connection instanceof HttpsURLConnection) {
                    // Para desarrollo - quitar en producción
                    ((HttpsURLConnection) connection).setHostnameVerifier((hostname, session) -> true);
                }

                connection.setRequestMethod("POST");
                connection.setRequestProperty("Content-Type", "application/json");
                connection.setRequestProperty("Accept", "application/json");
                connection.setConnectTimeout(TIMEOUT_MS);
                connection.setReadTimeout(TIMEOUT_MS);
                connection.setDoOutput(true);

                // Crear JSON con datos del evento háptico
                JSONObject jsonData = new JSONObject();
                jsonData.put("event_type", eventType);
                jsonData.put("controller", controller);
                jsonData.put("intensity", intensity);
                jsonData.put("timestamp", System.currentTimeMillis());
                jsonData.put("device", "meta_quest");

                String jsonInputString = jsonData.toString();

                // Enviar datos
                try (OutputStream os = connection.getOutputStream()) {
                    byte[] input = jsonInputString.getBytes("utf-8");
                    os.write(input, 0, input.length);
                }

                // Leer respuesta
                int responseCode = connection.getResponseCode();
                Log.d(TAG, "HTTP Response Code: " + responseCode);

                if (responseCode == HttpURLConnection.HTTP_OK ||
                        responseCode == HttpURLConnection.HTTP_CREATED) {

                    BufferedReader in = new BufferedReader(
                            new InputStreamReader(connection.getInputStream()));
                    String inputLine;
                    StringBuilder response = new StringBuilder();

                    while ((inputLine = in.readLine()) != null) {
                        response.append(inputLine);
                    }
                    in.close();

                    Log.d(TAG, "Server response: " + response.toString());

                    if (callback != null) {
                        callback.onSuccess(response.toString());
                    }
                } else {
                    String errorMsg = "HTTP error code: " + responseCode;
                    Log.e(TAG, errorMsg);
                    if (callback != null) {
                        callback.onError(errorMsg);
                    }
                }

            } catch (Exception e) {
                Log.e(TAG, "Error sending haptic event: " + e.getMessage(), e);
                if (callback != null) {
                    callback.onError("Network error: " + e.getMessage());
                }
            } finally {
                if (connection != null) {
                    connection.disconnect();
                }
            }
        }).start();
    }
}
