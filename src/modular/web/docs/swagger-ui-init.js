window.onload = function () {
  // Build a system
  const ui = SwaggerUIBundle({
    url: "./modular_swagger.yaml",
    dom_id: "#swagger-ui",
    defaultModelsExpandDepth: -1,
    deepLinking: true,
    presets: [SwaggerUIBundle.presets.apis, SwaggerUIStandalonePreset],
    plugins: [SwaggerUIBundle.plugins.DownloadUrl],
    layout: "StandaloneLayout",
  });

  window.ui = ui;
};
