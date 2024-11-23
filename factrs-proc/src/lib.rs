use syn::ItemImpl;

mod noise;
mod residual;
mod robust;
mod variable;

mod kw {
    syn::custom_keyword!(path);
}

enum BoxedTypes {
    Residual,
    Variable,
    Noise,
    Robust,
}

fn check_type(input: &ItemImpl) -> syn::Result<BoxedTypes> {
    let err = syn::Error::new_spanned(input, "Missing trait");
    let result = &input
        .trait_
        .as_ref()
        .ok_or(err.clone())?
        .1
        .segments
        .last()
        .ok_or(err)?
        .ident
        .to_string();

    if result.contains("Residual") {
        Ok(BoxedTypes::Residual)
    } else if result.contains("Variable") {
        Ok(BoxedTypes::Variable)
    } else if result.contains("Noise") {
        Ok(BoxedTypes::Noise)
    } else if result.contains("Robust") {
        Ok(BoxedTypes::Robust)
    } else {
        Err(syn::Error::new_spanned(input, "Not a valid trait"))
    }
}

#[proc_macro_attribute]
pub fn tag(
    _args: proc_macro::TokenStream,
    input: proc_macro::TokenStream,
) -> proc_macro::TokenStream {
    let input = syn::parse_macro_input!(input as ItemImpl);

    let trait_type = match check_type(&input) {
        Ok(syntax_tree) => syntax_tree,
        Err(err) => return proc_macro::TokenStream::from(err.to_compile_error()),
    };

    match trait_type {
        BoxedTypes::Residual => residual::tag(input),
        BoxedTypes::Variable => variable::tag(input),
        BoxedTypes::Noise => noise::tag(input),
        BoxedTypes::Robust => robust::tag(input),
    }
    .into()
}
