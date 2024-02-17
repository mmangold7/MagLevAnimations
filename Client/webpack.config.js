const path = require('path');
const webpack = require('webpack');

module.exports = (env) => {
    return {
        // Entry point of your application
        entry: './src/main.js', // Should be correct as 'src' is now in the root

        // Output configuration
        output: {
            // Output directory
            path: path.resolve(__dirname, 'wwwroot/js'), // Update this path

            // Output file name
            filename: 'bundle.js'
        },

        // Module rules
        module: {
            rules: [
                {
                    test: /\.js$/, // Transpile .js files
                    exclude: /node_modules/, // Don't transpile node_modules
                    use: {
                        loader: 'babel-loader', // Use Babel for transpilation
                        options: {
                            presets: ['@babel/preset-env'] // Preset for modern JavaScript
                        }
                    }
                }
            ]
        },

        // Development tools (source maps)
        devtool: 'source-map',

        // Mode (development or production)
        mode: 'development', // Change to 'production' when building for production

        // Additional options (e.g., for resolving modules)
        resolve: {
            extensions: ['.js'], // File types to process
        },

        plugins: [
            new webpack.DefinePlugin({
                'process.env.ENVIRONMENT': JSON.stringify(process.env.ENVIRONMENT)
            })
            //,new webpack.HotModuleReplacementPlugin()
        ]
    };
};