import React from "react";
import { BrowserRouter as Router, Routes, Route } from "react-router-dom";
import NavbarComponent from "./components/NavbarComponent";
import FooterComponent from "./components/FooterComponent";
import Home from "./pages/Home";
import About from "./pages/About";
import Test from "./pages/Test"

const App = () => {
  return (
    <Router>
      <NavbarComponent />
      <div className="container">
        <Routes>
          <Route path="/" element={<Home />} />
          <Route path="/about" element={<About />} />
          <Route path="/test" element={<Test />} />
        </Routes>
      </div>
      <FooterComponent />
    </Router>
  );
};

export default App;
