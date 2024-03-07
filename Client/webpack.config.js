const path = require('path');
const webpack = require('webpack');

module.exports = (env) => {
    return {
        entry: './src/main.js',

        output: {
            path: path.resolve(__dirname, 'wwwroot/js'),
            filename: 'bundle.js'
        },

        module: {
            rules: [
                {
                    test: /\.js$/,
                    exclude: /node_modules/,
                    use: {
                        loader: 'babel-loader',
                        options: {
                            presets: ['@babel/preset-env']
                        }
                    }
                }
            ]
        },

        devtool: 'source-map',
        mode: 'development',

        resolve: {
            extensions: ['.js']
        },

        plugins: [
            new webpack.DefinePlugin({
                'process.env.ENVIRONMENT': JSON.stringify(process.env.ENVIRONMENT)
            })
            //,new webpack.HotModuleReplacementPlugin()
        ]
    };
};